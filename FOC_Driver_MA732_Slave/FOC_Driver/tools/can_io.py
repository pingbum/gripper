# -*- coding: utf-8 -*-
import time
import threading
import sys
import json
import subprocess
from collections import deque
from pathlib import Path
from typing import Dict, Optional, Tuple
import can
from config import TX_TIMEOUT_S
from can_workers import CANWriterThread


def connection_options(interface, channel, bitrate):
    channel = channel.strip()
    if not channel:
        raise ValueError("Select or enter a CAN channel/serial port")
    options = dict(interface=interface, channel=channel, ignore_config=True)
    if interface == "socketcan":
        if not sys.platform.startswith("linux"):
            raise ValueError("SocketCAN requires Linux; use slcan for a Windows COM port")
        # Bit timing is owned by Linux (ip link / slcand), not Bus().
        # The interface's `fd on` does not enable FD reception on this socket.
        options.update(fd=True, receive_own_messages=False)
    elif interface == "slcan":
        if bitrate != 1_000_000:
            raise ValueError("This firmware requires CAN FD 1M/5M with BRS")
        # CANable 2.0-compatible FD firmware: python-can sends S8/Y5.
        # Generic Classic-only SLCAN firmware cannot operate this bus.
        options["timing"] = can.BitTimingFd(
            f_clock=80_000_000,
            nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=4,
            data_brp=1, data_tseg1=11, data_tseg2=4, data_sjw=4,
        )
    elif interface == "gs_usb":
        raise ValueError("python-can gs_usb is Classic-only; use FD-capable SocketCAN or SLCAN")
    else:
        raise ValueError(f"Unsupported interface: {interface}")
    return options

class CANBusManager:
    """
    python-can 래퍼: 연결/해제/송신
    """
    def __init__(self):
        self.bus: Optional[can.Bus] = None
        self.writer = None
        self.interface = None
        self.channel = None
        self.setup_warning = None

    def connect(self, interface: str, channel: str, bitrate: int, tx_brs: bool = True):
        if self.bus is not None:
            return
        self.bus = can.Bus(**connection_options(interface, channel, bitrate))
        self.interface, self.channel = interface, channel.strip()
        self.setup_warning = None
        if interface == "socketcan":
            try:
                # Jetson mttcan exposes this node on recent L4T kernels.
                tdc = (Path("/sys/class/net") / self.channel / "tdc_offset").read_text().strip()
            except OSError:
                pass  # Other SocketCAN controllers need not expose it.
            else:
                if "DBTP.tdc=0" in tdc:
                    self.setup_warning = (
                        f"{self.channel}: {tdc}. Jetson TX delay compensation is disabled; "
                        "5M FD+BRS transmission may fail even when RX works. "
                        "Configure/check Jetson TDC before sending commands.")
        self.writer = CANWriterThread(self.bus, bitrate_switch=tx_brs)
        self.writer.start()

    def disconnect(self):
        if self.bus is None:
            return
        try:
            if self.writer:
                self.writer.stop()
                self.writer.join(timeout=1.0)
            self.bus.shutdown()
            if self.writer:
                self.writer.join(timeout=1.0)
        finally:
            self.bus = None
            self.writer = None

    def send_ext(self, arbitration_id: int, data: bytes, timeout: float = TX_TIMEOUT_S):
        """Queue a command without blocking the GUI. Errors/stats come from writer."""
        if self.bus is None or self.writer is None:
            raise RuntimeError("CAN bus not connected")
        self.writer.enqueue(arbitration_id, data, timeout)

    def resume_tx(self, expected_writer=None):
        """Read kernel state without changing can0; resume only fresh requests."""
        writer = self.writer
        if expected_writer is not None and writer is not expected_writer:
            raise RuntimeError("Connection changed before state check")
        if self.bus is None or writer is None:
            raise RuntimeError("CAN bus not connected")
        if self.interface != "socketcan":
            raise RuntimeError("State check requires SocketCAN; reconnect the adapter instead")
        generation = writer.transport_generation()
        result = subprocess.run(
            ["ip", "-details", "-json", "link", "show", "dev", self.channel],
            check=True, capture_output=True, text=True, timeout=2,
        )
        info = json.loads(result.stdout)[0].get("linkinfo", {}).get("info_data", {})
        state = info.get("state", "UNKNOWN")
        if state != "ERROR-ACTIVE":
            raise RuntimeError(f"{self.channel} state={state}; TX remains paused")
        if self.writer is not writer or self.bus is None:
            raise RuntimeError("Connection changed during state check")
        writer.resume(generation)
        return f"CAN state={state}; TX ready for new requests (reference remains stopped)"

class CANReaderThread(threading.Thread):
    """
    수신 스레드: 각 드라이버의 최신 상태를 독립적으로 보관한다.
    스캔/파라미터 응답은 기존 대기 경로로 전달한다.
    """
    def __init__(
        self,
        bus: can.Bus,
        stop_event: threading.Event,
        scan_hits: Optional[set] = None,
        scan_lock: Optional[threading.Lock] = None,
        scan_func_id: int = 0x07,
        read_watch: Optional[dict] = None,
        read_lock: Optional[threading.Lock] = None,
        on_error=None,
        on_warning=None,
    ):
        super().__init__(daemon=True)
        self.bus = bus
        self.stop_event = stop_event
        self.scan_hits = scan_hits
        self.scan_lock = scan_lock
        self.scan_func_id = scan_func_id
        self.read_watch = read_watch
        self.read_lock = read_lock
        self.on_error = on_error
        self.on_warning = on_warning
        self._rx_count = 0
        self._count_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._latest_status: Dict[int, Tuple[float, bytes]] = {}
        self.recorder = None
        self.last_error = None
        self.last_warning = None
        self.warning_count = 0
        self._warning_key = None
        self._notice_lock = threading.Lock()
        self._notices = deque(maxlen=16)

    def take_latest_status(self, driver_id: int) -> Optional[Tuple[float, bytes]]:
        """Consume this driver's newest sample once; retain other drivers' samples."""
        with self._status_lock:
            return self._latest_status.pop(driver_id, None)

    def get_and_reset_rx_count(self) -> int:
        with self._count_lock:
            count = self._rx_count
            self._rx_count = 0
            return count

    def _fail(self, reason):
        self.last_error = reason
        if self.on_error is not None:
            self.on_error()

    def take_notices(self):
        with self._notice_lock:
            notices = list(self._notices)
            self._notices.clear()
            return notices

    def _warn(self, reason, key, pause_tx=False):
        self.warning_count += 1
        newly_paused = False
        if pause_tx and self.on_warning is not None:
            newly_paused = bool(self.on_warning())
        # Counter-byte changes alone are not new controller states.
        if key != self._warning_key or newly_paused:
            self._warning_key = key
            self.last_warning = reason
            with self._notice_lock:
                self._notices.append(("warning", reason))

    def _recovered(self):
        if self._warning_key is None:
            return
        self._warning_key = None
        self.last_warning = None
        with self._notice_lock:
            self._notices.append(("state", "CAN ERROR-ACTIVE reported"))

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.05)
            except Exception as exc:
                self._fail(str(exc))
                break
            if msg is None:
                continue
            recorder = self.recorder
            if recorder is not None:
                recorder.offer(msg)
            if getattr(msg, "is_error_frame", False):
                # Linux CAN error classes (include/uapi/linux/can/error.h).
                # Bus-off is fatal. RX-only error-passive is a warning, not
                # a transmit failure: an error-passive CAN node can still TX.
                if msg.arbitration_id & 0x40:  # CAN_ERR_BUSOFF
                    self._fail("CAN BUS-OFF: TX stopped; recover can0 and check timing/ACK/TDC")
                    break
                if (msg.arbitration_id & 0x04 and len(msg.data) > 1
                        and msg.data[1] & 0x30):  # RX/TX error-passive
                    sides = []
                    if msg.data[1] & 0x10:
                        sides.append("RX")
                    if msg.data[1] & 0x20:
                        sides.append("TX")
                    pause_tx = bool(msg.data[1] & 0x20)
                    action = ("TX paused; RX continues." if pause_tx else
                              "RX warning; TX remains available.")
                    self._warn(f"CAN ERROR-PASSIVE ({'/'.join(sides)}, "
                               f"ctrl=0x{msg.data[1]:02X}, data={bytes(msg.data).hex(' ')}): "
                               f"{action} Check can0 error counters/timing.",
                               msg.data[1] & 0x30, pause_tx=pause_tx)
                elif (msg.arbitration_id & 0x04 and len(msg.data) > 1
                      and msg.data[1] & 0x40):  # CAN_ERR_CRTL_ACTIVE
                    self._recovered()
                continue
            if not getattr(msg, "is_extended_id", False):
                continue
            if getattr(msg, "is_remote_frame", False) or not getattr(msg, "is_rx", True):
                continue
            data = bytes(getattr(msg, "data", b""))
            if len(data) not in (1, 4, 8):
                continue
            with self._count_lock:
                self._rx_count += 1
            if self.read_watch is not None and self.read_lock is not None:
                eid = msg.arbitration_id
                with self.read_lock:
                    watch_eid = self.read_watch.get("eid")
                    if watch_eid is not None and int(watch_eid) == int(eid):
                        self.read_watch["data"] = bytes(data)
                        self.read_watch["ts"] = getattr(msg, "timestamp", time.time())
            if self.scan_hits is not None and self.scan_lock is not None:
                eid = msg.arbitration_id
                mode_id = (eid >> 8) & 0xFF
                if mode_id == (self.scan_func_id & 0xFF):
                    driver_id = eid & 0xFF
                    with self.scan_lock:
                        self.scan_hits.add(int(driver_id))
            # Status uses mode 0 (EID equals the 8-bit driver ID), with 8 bytes.
            # Commands/responses sharing the low ID byte must not replace it.
            if 0 <= msg.arbitration_id <= 0xFF and len(data) == 8:
                with self._status_lock:
                    self._latest_status[msg.arbitration_id] = (time.monotonic(), data)
                # Discover broadcasting motors without transmitting a scan.
                if self.scan_hits is not None and self.scan_lock is not None:
                    with self.scan_lock:
                        self.scan_hits.add(msg.arbitration_id)
