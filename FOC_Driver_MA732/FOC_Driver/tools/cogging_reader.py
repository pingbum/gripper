"""Cogging measurement/status/data protocol, independent of Qt.

The bus RX path must have a single owner. Data is mechanical-direction iq_ref
[A], not measured q-axis current or torque. Encoder correction is unrelated.
"""
import math
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import can


MODE_MEASURE = 0x30
MODE_COMPENSATION = 0x31
MODE_DATA = 0x32  # No 0x80 read flag!
BIN_COUNT = 1440
SELECTOR_FILES = {
    0x00: "cogging_cw_sweep_1.txt",
    0x01: "cogging_cw_sweep_2.txt",
    0x02: "cogging_cw_sweep_3.txt",
    0x40: "cogging_ccw_sweep_1.txt",
    0x41: "cogging_ccw_sweep_2.txt",
    0x42: "cogging_ccw_sweep_3.txt",
    0x80: "cogging_lut.txt",
}


class CoggingCancelled(Exception):
    pass


@dataclass(frozen=True)
class CoggingStatus:
    state: int
    sweeps: int
    version: int
    bin_count: int
    flags: int

    @property
    def has_raw(self):
        return bool(self.flags & 0x01)

    @property
    def lut_valid(self):
        return bool(self.flags & 0x02)

    @property
    def processing(self):
        return bool(self.flags & 0x04)

    @property
    def flash_saved(self):
        return bool(self.flags & 0x08)

    @property
    def flash_failed(self):
        return bool(self.flags & 0x10)

    @classmethod
    def decode(cls, payload):
        if len(payload) != 8:
            raise ValueError("Cogging status must be exactly eight bytes")
        operation, state, sweeps, version, bins, flags, reserved = struct.unpack(">BBBBHBB", payload)
        if operation != 0 or state not in (0, 1, 2, 3) or reserved != 0:
            raise ValueError("Invalid cogging status response")
        if (sweeps, version, bins) != (3, 1, BIN_COUNT):
            raise ValueError(f"Unsupported cogging format: version={version}, sweeps={sweeps}, bins={bins}")
        return cls(state, sweeps, version, bins, flags)


@dataclass(frozen=True)
class CoggingCapture:
    driver_id: int
    status: CoggingStatus
    series: dict[int, tuple[float, ...]]


def pack_measurement_rpm(rpm):
    if not math.isfinite(rpm) or (rpm != 0 and not 1 <= rpm <= 20):
        raise ValueError("Measurement RPM must be 1..20, or 0 to abort")
    return struct.pack(">i", round(rpm * 100.0))


class CoggingReader:
    def __init__(self, bus, driver_id=0, timeout=0.35, retries=5, stop_event=None, on_retry=None):
        if not isinstance(driver_id, int) or not 0 <= driver_id <= 255:
            raise ValueError("Driver ID must be an integer in 0..255")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Timeout must be finite and positive")
        if not isinstance(retries, int) or retries < 0:
            raise ValueError("Retries must be a non-negative integer")
        self.bus, self.driver_id = bus, driver_id
        self.timeout, self.retries = timeout, retries
        self.stop_event = stop_event if stop_event is not None else threading.Event()
        self.on_retry = on_retry

    def _check_cancelled(self):
        if self.stop_event.is_set():
            raise CoggingCancelled()

    def _send(self, mode, payload):
        message = can.Message(arbitration_id=(mode << 8) | self.driver_id,
                              is_extended_id=True, is_fd=True, bitrate_switch=False,
                              data=payload, check=True)
        self.bus.send(message, timeout=min(self.timeout, 0.02))

    def start_measurement(self, rpm):
        self._check_cancelled()
        if rpm == 0:
            raise ValueError("Use abort_measurement() for zero RPM")
        self._send(MODE_MEASURE, pack_measurement_rpm(rpm))

    def abort_measurement(self):
        # An abort must still be attempted after a cancelled request or close.
        self._send(MODE_MEASURE, pack_measurement_rpm(0))

    def set_compensation(self, enabled):
        self._check_cancelled()
        self._send(MODE_COMPENSATION, bytes([bool(enabled)]))

    def _request(self, payload, label, data_reply=False):
        eid = (MODE_DATA << 8) | self.driver_id
        for attempt in range(self.retries + 1):
            self._check_cancelled()
            self._send(MODE_DATA, payload)
            deadline = time.monotonic() + self.timeout
            while True:
                self._check_cancelled()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                message = self.bus.recv(timeout=min(remaining, 0.02))
                if message is None:
                    continue
                if message.is_error_frame:
                    if message.arbitration_id & 0x40:
                        raise can.CanOperationError("CAN BUS-OFF during cogging read")
                    continue
                if (not message.is_rx or message.is_remote_frame or not message.is_extended_id
                        or not message.is_fd or message.bitrate_switch
                        or message.arbitration_id != eid or message.dlc != 8
                        or len(message.data) != 8):
                    continue
                response = bytes(message.data)
                if data_reply:
                    if response[:4] != payload[:4]:
                        continue
                elif response[0] != 0:
                    continue
                return response
            if attempt < self.retries and self.on_retry is not None:
                self.on_retry(label, attempt + 1, self.retries)
        raise TimeoutError(f"Cogging {label}: no matching reply after {self.retries + 1} attempts")

    def read_status(self):
        return CoggingStatus.decode(self._request(bytes(8), "status"))

    @staticmethod
    def _can_download(status, final_only):
        if status.state in (1, 3) or status.processing or not status.lut_valid:
            return False
        return final_only or (status.state == 2 and status.has_raw)

    def download(self, final_only=False, on_status=None, on_progress=None):
        initial = self.read_status()
        if on_status is not None:
            on_status(initial)
        if not self._can_download(initial, final_only):
            raise RuntimeError("Cogging data unavailable: full download needs state 2/raw data; "
                               "final LUT needs a valid LUT and completed processing")
        selectors = (0x80,) if final_only else tuple(SELECTOR_FILES)
        total = len(selectors) * initial.bin_count
        series, done = {}, 0
        latest = initial
        for selector in selectors:
            values = []
            for bin_index in range(initial.bin_count):
                request = struct.pack(">BBHI", 1, selector, bin_index, 0)
                payload = self._request(request, f"selector 0x{selector:02X}/bin {bin_index}", True)
                _, _, _, value = struct.unpack(">BBHf", payload)
                values.append(value)
                done += 1
                if on_progress is not None:
                    on_progress(done, total)
            series[selector] = tuple(values)
            # Catch reset/re-measurement observed between sweeps. There is no
            # generation ID, so external resets/re-measurement still require a
            # manual restart even if their transient state was not observed.
            latest = self.read_status()
            if on_status is not None:
                on_status(latest)
            if (not self._can_download(latest, final_only)
                    or latest.state != initial.state or latest.has_raw != initial.has_raw):
                raise RuntimeError("Cogging state changed during download; restart collection")
        self._check_cancelled()
        return CoggingCapture(self.driver_id, latest, series)


def write_cogging_files(directory, capture):
    """Create a new output folder containing complete, headerless text files."""
    keys = set(capture.series)
    if keys not in (set(SELECTOR_FILES), {0x80}):
        raise ValueError("Cannot save an incomplete cogging capture")
    if any(len(values) != BIN_COUNT for values in capture.series.values()):
        raise ValueError("Every cogging series must contain 1440 bins")
    directory = Path(directory)
    directory.mkdir()  # Never merge with or overwrite an existing capture.
    created = []
    try:
        for selector, values in capture.series.items():
            path = directory / SELECTOR_FILES[selector]
            with path.open("x", encoding="utf-8", newline="\n") as stream:
                created.append(path)
                stream.writelines(f"{value:.9g}\n" for value in values)
    except BaseException:
        for path in created:
            path.unlink(missing_ok=True)
        directory.rmdir()
        raise
    return directory
