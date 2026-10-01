"""CAN transmission and raw recording, independent of Qt and its event loop."""
import csv
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

import can

from config import MAX_REF_SEND_HZ, RECORD_QUEUE_SIZE, TX_QUEUE_SIZE, TX_TIMEOUT_S
from protocol import build_ext_id, pack_command_payload
from signal_gen import sine, square, triangle


@dataclass(frozen=True)
class ReferenceCommand:
    driver_id: int
    mode_id: int
    shape: str
    amplitude: float
    frequency: float

    def __post_init__(self):
        if self.shape not in ("Sine", "Triangle", "Square"):
            raise ValueError("Unknown reference shape")
        if not (math.isfinite(self.amplitude) and math.isfinite(self.frequency)):
            raise ValueError("Reference parameters must be finite")
        if self.amplitude < 0 or self.frequency <= 0:
            raise ValueError("Invalid reference amplitude/frequency")
        # Validate both peaks before starting a waveform, not partway through it.
        pack_command_payload(self.mode_id, self.amplitude)
        pack_command_payload(self.mode_id, -self.amplitude)

    def __call__(self, elapsed):
        waveform = {"Sine": sine, "Triangle": triangle, "Square": square}[self.shape]
        value = waveform(elapsed, self.amplitude, self.frequency)
        return (build_ext_id(self.mode_id, self.driver_id),
                pack_command_payload(self.mode_id, value), value)


class CANWriterThread(threading.Thread):
    """Single TX owner. GUI enqueues commands; periodic packets never accumulate.

    A periodic factory takes elapsed seconds and returns (EID, payload, plot_value).
    A future packed control command can use the same interface.
    """
    def __init__(self, bus):
        super().__init__(name="can-tx", daemon=True)
        self.bus = bus
        self.commands = queue.Queue(maxsize=TX_QUEUE_SIZE)
        self.stop_event = threading.Event()
        self.wake = threading.Event()
        self.lock = threading.Lock()
        self.factory = None
        self.period = 0.001
        self.origin = 0.0
        self.generation = 0
        self.sent = 0
        self.skipped = 0
        self.errors = deque(maxlen=16)
        self.latest_reference = None

    def enqueue(self, eid, payload, timeout=TX_TIMEOUT_S):
        message = can.Message(arbitration_id=eid, is_extended_id=True,
                              data=bytes(payload), check=True)
        with self.lock:
            if self.stop_event.is_set():
                raise RuntimeError("CAN transmitter is stopped")
            try:
                self.commands.put_nowait((message, timeout))
            except queue.Full:
                raise RuntimeError("TX queue full; command was not queued") from None
        self.wake.set()

    def set_periodic(self, factory, rate_hz):
        if not math.isfinite(rate_hz) or not 1 <= rate_hz <= MAX_REF_SEND_HZ:
            raise ValueError(f"TX rate must be 1..{MAX_REF_SEND_HZ} Hz")
        with self.lock:
            if self.stop_event.is_set():
                raise RuntimeError("CAN transmitter is stopped")
            self.factory = factory
            self.period = 1.0 / rate_hz
            self.origin = time.perf_counter()
            self.generation += 1
            self.latest_reference = None
        self.wake.set()

    def update_periodic(self, factory):
        with self.lock:
            if self.factory is not None:
                self.factory = factory

    def stop_periodic(self):
        with self.lock:
            self.factory = None
            self.generation += 1
            self.latest_reference = None
        self.wake.set()

    def stop(self):
        with self.lock:
            self.stop_event.set()
            self.factory = None
            self.generation += 1
        self.wake.set()

    def take_reference(self):
        with self.lock:
            sample, self.latest_reference = self.latest_reference, None
            return sample

    def take_stats(self):
        with self.lock:
            result = self.sent, self.skipped, list(self.errors)
            self.sent = self.skipped = 0
            self.errors.clear()
            return result

    def periodic_active(self):
        with self.lock:
            return self.factory is not None

    def _send(self, message, timeout):
        if self.stop_event.is_set():
            return False
        try:
            self.bus.send(message, timeout=timeout)
        except Exception as exc:
            with self.lock:
                self.errors.append(str(exc))
            # Drop pending commands and stop reference on a transport error.
            # A reconnect creates a new worker, so stale commands never replay.
            self.stop()
            return False
        with self.lock:
            self.sent += 1
        return True

    def run(self):
        generation = -1
        deadline = 0.0
        while not self.stop_event.is_set():
            self.wake.clear()
            try:
                message, timeout = self.commands.get_nowait()
            except queue.Empty:
                pass
            else:
                if not self._send(message, timeout):
                    break

            with self.lock:
                factory, period, origin, current = (
                    self.factory, self.period, self.origin, self.generation)
            if current != generation:
                generation, deadline = current, time.perf_counter()
            now = time.perf_counter()
            if factory is not None and now >= deadline:
                try:
                    eid, payload, value = factory(now - origin)
                    message = can.Message(arbitration_id=eid, is_extended_id=True,
                                          data=payload, check=True)
                except Exception as exc:
                    with self.lock:
                        self.errors.append(str(exc))
                    self.stop_periodic()
                    continue
                with self.lock:
                    valid = generation == self.generation and self.factory is not None
                if valid and self._send(message, TX_TIMEOUT_S):
                    with self.lock:
                        if generation == self.generation:
                            self.latest_reference = (time.monotonic(), value)
                finished = time.perf_counter()
                missed = max(0, int((finished - deadline) / period))
                with self.lock:
                    self.skipped += missed
                # Never catch up by sending a burst of old setpoints.
                deadline = max(deadline + period, now + period)
                if deadline <= finished:
                    deadline = finished + period
            if not self.commands.empty():
                continue
            if factory is None:
                self.wake.wait(0.05)
            else:
                delay = max(0.0, deadline - time.perf_counter())
                if delay > 0.002:
                    # Low TX rates must not delay manual commands or Stop Ref.
                    self.wake.wait(min(0.05, delay - 0.001))
                else:
                    # Python >=3.11 uses a high-resolution Windows sleep timer.
                    time.sleep(delay)


class CaptureWriter(threading.Thread):
    """Bounded raw RX queue. Slow storage is reported as drops, never blocks RX."""
    def __init__(self, path, capacity=RECORD_QUEUE_SIZE):
        super().__init__(name="can-csv", daemon=True)
        self.path = str(path)
        self.queue = queue.Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.done = threading.Event()
        self.written = 0
        self.dropped = 0
        self.error = None

    def offer(self, message):
        with self.lock:
            if self.stopping.is_set():
                return
            row = (message.timestamp, time.monotonic(), f"{message.arbitration_id:08X}",
                   int(message.is_extended_id), int(message.is_fd),
                   int(message.is_error_frame), int(message.is_remote_frame),
                   message.dlc, bytes(message.data).hex())
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                self.dropped += 1

    def stop(self):
        with self.lock:
            self.stopping.set()

    def snapshot(self):
        with self.lock:
            return self.written, self.dropped, self.error

    def run(self):
        try:
            # Exclusive creation protects previous recordings even with a stale dialog.
            with open(self.path, "x", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(("timestamp", "host_monotonic", "arbitration_id",
                                 "extended", "fd", "error", "remote", "dlc", "data_hex"))
                last_flush = time.monotonic()
                while not self.stopping.is_set() or not self.queue.empty():
                    try:
                        row = self.queue.get(timeout=0.05)
                    except queue.Empty:
                        row = None
                    if row is not None:
                        writer.writerow(row)
                        with self.lock:
                            self.written += 1
                    if time.monotonic() - last_flush >= 1.0:
                        handle.flush()
                        last_flush = time.monotonic()
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.stopping.set()
        finally:
            self.done.set()
