"""Read the active, filtered encoder LUT without changing motor state.

This module has no Qt dependency. Each reader exclusively owns its bus's RX
path; do not attach the monitor's CANReaderThread to the same bus object.
"""
import csv
import math
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import can


READ_LUT_METADATA = 0xC0
READ_LUT_SAMPLE = 0xC1
# The current firmware exposes indices 0..9999. A count is not a calibration flag.
MAX_LUT_SAMPLES = 10_000


class LUTCancelled(Exception):
    """The user cancelled; no partial capture is returned."""


@dataclass(frozen=True)
class LUTMetadata:
    sample_count: int
    direction: float


@dataclass(frozen=True)
class LUTCapture:
    driver_id: int
    metadata: LUTMetadata
    offsets: tuple[float, ...]


class LUTReader:
    def __init__(self, bus, driver_id=0, timeout=0.1, retries=3,
                 stop_event=None, on_retry=None):
        if not isinstance(driver_id, int) or not 0 <= driver_id <= 255:
            raise ValueError("Driver ID must be an integer in 0..255")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Timeout must be finite and positive")
        if not isinstance(retries, int) or retries < 0:
            raise ValueError("Retries must be a non-negative integer")
        self.bus = bus
        self.driver_id = driver_id
        self.timeout = timeout
        self.retries = retries
        self.stop_event = stop_event if stop_event is not None else threading.Event()
        self.on_retry = on_retry

    def _check_cancelled(self):
        if self.stop_event.is_set():
            raise LUTCancelled()

    def _request(self, mode, index=None):
        eid = (mode << 8) | self.driver_id
        # Short legal requests cannot be confused with eight-byte replies, even
        # on adapters that report an echo as RX. SocketCAN also marks local
        # transmissions (including other sockets) with is_rx=False.
        payload = b"" if index is None else struct.pack(">I", index)
        request = can.Message(arbitration_id=eid, data=payload,
                              is_extended_id=True, is_fd=True,
                              bitrate_switch=False, check=True)
        label = "metadata" if index is None else f"index {index}"
        for attempt in range(self.retries + 1):
            self._check_cancelled()
            # Send errors are transport failures, not response timeouts.
            self.bus.send(request, timeout=min(self.timeout, 0.02))
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
                    if message.arbitration_id & 0x40:  # CAN_ERR_BUSOFF
                        raise RuntimeError("CAN BUS-OFF during LUT read")
                    continue
                if (not message.is_rx or message.is_remote_frame
                        or not message.is_extended_id or not message.is_fd
                        or message.bitrate_switch or message.arbitration_id != eid
                        or message.dlc != 8 or len(message.data) != 8):
                    continue
                returned_index, value = struct.unpack(">If", message.data)
                if index is not None and returned_index != index:
                    continue
                # NaN/Inf and zeros are diagnostic data, never discarded.
                return returned_index, value
            if attempt < self.retries and self.on_retry is not None:
                self.on_retry(label, attempt + 1, self.retries)
        raise TimeoutError(f"LUT {label}: no matching response after "
                           f"{self.retries + 1} attempts "
                           f"({self.timeout * 1000:g} ms each)")

    def read_metadata(self):
        count, direction = self._request(READ_LUT_METADATA)
        if not 1 <= count <= MAX_LUT_SAMPLES:
            raise ValueError(f"Unsupported LUT sample count: {count} "
                             f"(expected 1..{MAX_LUT_SAMPLES})")
        return LUTMetadata(count, direction)

    def read_all(self, on_metadata=None, on_progress=None):
        metadata = self.read_metadata()
        if on_metadata is not None:
            on_metadata(metadata)
        offsets = []
        for index in range(metadata.sample_count):
            _, value = self._request(READ_LUT_SAMPLE, index)
            offsets.append(value)
            if on_progress is not None:
                on_progress(index + 1, metadata.sample_count)
        self._check_cancelled()
        return LUTCapture(self.driver_id, metadata, tuple(offsets))


def write_lut_csv(path, capture):
    """Save a complete capture, preserving every sample including NaN/Inf.

    Existing files are never overwritten. Remove an incomplete file if writing
    fails, so it cannot be mistaken for a successful capture.
    """
    count = capture.metadata.sample_count
    if count <= 0 or len(capture.offsets) != count:
        raise ValueError("Cannot save an incomplete LUT capture")
    path = Path(path)
    stream = path.open("x", newline="", encoding="utf-8")
    try:
        with stream:
            writer = csv.writer(stream)
            writer.writerow(("index", "mechanical_deg", "offset_electrical_deg"))
            for index, value in enumerate(capture.offsets):
                writer.writerow((index, index * 360.0 / count, value))
    except BaseException:
        path.unlink(missing_ok=True)
        raise
