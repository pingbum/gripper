# -*- coding: utf-8 -*-
import time
import threading
import queue
from typing import Optional, Tuple
import can

class CANBusManager:
    """
    python-can 래퍼: 연결/해제/송신
    """
    def __init__(self):
        self.bus: Optional[can.Bus] = None
        self._send_lock = threading.Lock()

    def connect(self, interface: str, channel: str, bitrate: int):
        if self.bus is not None:
            return
        if interface == "gs_usb":
            # candleLight: channel은 정수 인덱스
            self.bus = can.Bus(interface="gs_usb", channel=int(channel), bitrate=bitrate)
        elif interface == "slcan":
            # slcan: channel은 'COMx'
            self.bus = can.Bus(interface="slcan", channel=channel, bitrate=bitrate)
        elif interface == "socketcan":
            # 리눅스 표준인 can0, can1 사용
            self.bus = can.Bus(interface="socketcan", channel=channel, bitrate=bitrate)
        else:
            raise ValueError(f"Unsupported interface: {interface}")

    def disconnect(self):
        if self.bus is None:
            return
        try:
            self.bus.shutdown()
        finally:
            self.bus = None

    def send_ext(self, arbitration_id: int, data: bytes, timeout: float = 0.2):
        if self.bus is None:
            raise RuntimeError("CAN bus not connected")
        msg = can.Message(arbitration_id=arbitration_id, is_extended_id=True, data=data)
        with self._send_lock:
            self.bus.send(msg, timeout=timeout)

class CANReaderThread(threading.Thread):
    """
    수신 스레드: 확장 프레임만 큐에 넣는다.
    큐 항목: (timestamp, arbitration_id, data_bytes)
    """
    def __init__(
        self,
        bus: can.Bus,
        out_queue: "queue.Queue[Tuple[float,int,bytes]]",
        stop_event: threading.Event,
        scan_hits: Optional[set] = None,
        scan_lock: Optional[threading.Lock] = None,
        scan_func_id: int = 0x07,
        read_watch: Optional[dict] = None,
        read_lock: Optional[threading.Lock] = None,
    ):
        super().__init__(daemon=True)
        self.bus = bus
        self.q = out_queue
        self.stop_event = stop_event
        self.scan_hits = scan_hits
        self.scan_lock = scan_lock
        self.scan_func_id = scan_func_id
        self.read_watch = read_watch
        self.read_lock = read_lock
        self._rx_count = 0
        self._count_lock = threading.Lock()

    def get_and_reset_rx_count(self) -> int:
        with self._count_lock:
            count = self._rx_count
            self._rx_count = 0
        return count

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.05)
            except Exception:
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            if not getattr(msg, "is_extended_id", False):
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
                if watch_eid is not None and int(watch_eid) == int(eid) and len(data) >= 1:
                    with self.read_lock:
                        self.read_watch["data"] = bytes(data)
                        self.read_watch["ts"] = getattr(msg, "timestamp", time.time())
            if self.scan_hits is not None and self.scan_lock is not None:
                eid = msg.arbitration_id
                mode_id = (eid >> 8) & 0xFF
                if mode_id == (self.scan_func_id & 0xFF):
                    driver_id = eid & 0xFF
                    with self.scan_lock:
                        self.scan_hits.add(int(driver_id))
            ts = time.time()
            try:
                self.q.put_nowait((ts, msg.arbitration_id, data))
            except queue.Full:
                # 최신 것 위주로 보기 위해 오래된 것 버림
                try:
                    self.q.get_nowait()
                except Exception:
                    pass
                try:
                    self.q.put_nowait((ts, msg.arbitration_id, data))
                except Exception:
                    pass
