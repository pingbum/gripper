# -*- coding: utf-8 -*-
import time
import threading
import sys
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
        options["receive_own_messages"] = False
    elif interface == "slcan":
        options.update(bitrate=bitrate)
    elif interface == "gs_usb":
        options.update(channel=int(channel), bitrate=bitrate)
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

    def connect(self, interface: str, channel: str, bitrate: int):
        if self.bus is not None:
            return
        self.bus = can.Bus(**connection_options(interface, channel, bitrate))
        self.writer = CANWriterThread(self.bus)
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
    ):
        super().__init__(daemon=True)
        self.bus = bus
        self.stop_event = stop_event
        self.scan_hits = scan_hits
        self.scan_lock = scan_lock
        self.scan_func_id = scan_func_id
        self.read_watch = read_watch
        self.read_lock = read_lock
        self._rx_count = 0
        self._count_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._latest_status: Dict[int, Tuple[float, bytes]] = {}
        self.recorder = None
        self.last_error = None

    def take_latest_status(self, driver_id: int) -> Optional[Tuple[float, bytes]]:
        """Consume this driver's newest sample once; retain other drivers' samples."""
        with self._status_lock:
            return self._latest_status.pop(driver_id, None)

    def get_and_reset_rx_count(self) -> int:
        with self._count_lock:
            count = self._rx_count
            self._rx_count = 0
        return count

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.05)
            except Exception as exc:
                self.last_error = str(exc)
                break
            if msg is None:
                continue
            recorder = self.recorder
            if recorder is not None:
                recorder.offer(msg)
            if not getattr(msg, "is_extended_id", False):
                continue
            if getattr(msg, "is_error_frame", False) or getattr(msg, "is_remote_frame", False):
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
