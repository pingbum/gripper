"""Offline cogging wire protocol, complete-file export and GUI lifecycle."""
import math
import os
from pathlib import Path
import queue
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import can
from PyQt5 import QtCore, QtWidgets
from cogging_LUT import CoggingWindow
from cogging_reader import (BIN_COUNT, SELECTOR_FILES, CoggingCancelled, CoggingCapture,
                            CoggingReader, CoggingStatus, pack_measurement_rpm,
                            write_cogging_files)


def status_payload(state=2, flags=0x0B):
    return struct.pack(">BBBBHBB", 0, state, 3, 1, BIN_COUNT, flags, 0)


def frame(eid, data, **flags):
    options = dict(arbitration_id=eid, data=data, is_extended_id=True,
                   is_fd=True, bitrate_switch=False)
    options.update(flags)
    return can.Message(**options)


class FirmwareBus:
    def __init__(self, state=2, flags=0x0B):
        self.state, self.flags = state, flags
        self.sent = []
        self.pending = queue.Queue()
        self.closed = False

    def value(self, selector, index):
        return (index - 720) / 1000.0 + selector / 1000.0

    def send(self, message, timeout):
        self.sent.append(message)
        mode = message.arbitration_id >> 8
        if mode == 0x32:
            if message.data[0] == 0:
                payload = status_payload(self.state, self.flags)
            else:
                _, selector, index, _ = struct.unpack(">BBHI", message.data)
                payload = struct.pack(">BBHf", 1, selector, index, self.value(selector, index))
            self.pending.put(frame(message.arbitration_id, payload))
        elif mode == 0x30:
            rpm, = struct.unpack(">i", message.data[:4])
            self.state, self.flags = (1, 0) if rpm else (0, 0)

    def recv(self, timeout):
        try:
            return self.pending.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        self.closed = True


class CoggingProtocolTests(unittest.TestCase):
    def test_status_flags_including_flash_failure_and_boot_lut(self):
        failed = CoggingStatus.decode(status_payload(2, 0x13))
        self.assertTrue(failed.has_raw and failed.lut_valid and failed.flash_failed)
        self.assertFalse(failed.processing or failed.flash_saved)
        boot = CoggingStatus.decode(status_payload(0, 0x0A))
        self.assertTrue(boot.lut_valid and boot.flash_saved)
        self.assertFalse(boot.has_raw)
        with self.assertRaisesRegex(ValueError, "Unsupported"):
            CoggingStatus.decode(bytes([0, 2, 3, 2]) + status_payload()[4:])

    def test_all_10080_bins_are_sequential_unscaled_brs_off(self):
        bus = FirmwareBus(flags=0x13)  # Flash failure must not prevent RAM download.
        capture = CoggingReader(bus, driver_id=9).download()
        self.assertEqual(tuple(capture.series), tuple(SELECTOR_FILES))
        self.assertEqual(sum(map(len, capture.series.values())), 10080)
        requests = [m for m in bus.sent if m.data[0] == 1]
        self.assertEqual(len(requests), 10080)
        for selector_number, selector in enumerate(SELECTOR_FILES):
            for index in (0, 576, 1439):
                self.assertEqual(bytes(requests[selector_number * BIN_COUNT + index].data),
                                 struct.pack(">BBHI", 1, selector, index, 0))
                expected, = struct.unpack(">f", struct.pack(">f", bus.value(selector, index)))
                self.assertEqual(capture.series[selector][index], expected)
        self.assertTrue(capture.status.flash_failed)
        for message in bus.sent:
            self.assertEqual(message.arbitration_id, 0x3209)
            self.assertTrue(message.is_extended_id and message.is_fd)
            self.assertFalse(message.bitrate_switch)
            self.assertEqual(message.dlc, 8)

    def test_wrong_id_flags_prefix_and_tx_echo_are_ignored_even_for_zero_values(self):
        class NoisyBus(FirmwareBus):
            def value(self, selector, index):
                return 0.0

            def send(self, message, timeout):
                if message.data[0] == 1:
                    eid, data = message.arbitration_id, bytes(message.data)
                    self.pending.put(frame(eid, data, is_rx=False))
                    self.pending.put(frame(eid + 1, data))
                    self.pending.put(frame(eid, data, is_fd=False))
                    self.pending.put(frame(eid, data, bitrate_switch=True))
                    self.pending.put(frame(eid, data, is_extended_id=False))
                    self.pending.put(frame(eid, data, dlc=4))
                    self.pending.put(frame(eid, bytes([0]) + data[1:]))
                    self.pending.put(frame(eid, bytes([1, data[1] ^ 0x40]) + data[2:]))
                    self.pending.put(frame(eid, data[:2] + bytes([0xFF, 0xFF]) + data[4:]))
                super().send(message, timeout)

        result = CoggingReader(NoisyBus()).download(final_only=True)
        self.assertEqual(result.series[0x80], (0.0,) * BIN_COUNT)

    def test_retries_same_bin_and_exhaustion_fails_without_advancing(self):
        class DropBus(FirmwareBus):
            def __init__(self, drops):
                super().__init__()
                self.drops = drops

            def send(self, message, timeout):
                if message.data[0] == 1 and self.drops:
                    self.drops -= 1
                    self.sent.append(message)
                    return
                super().send(message, timeout)

        bus = DropBus(1)
        retries = []
        CoggingReader(bus, timeout=0.002, on_retry=lambda *r: retries.append(r)).download(final_only=True)
        data = [bytes(m.data) for m in bus.sent if m.data[0] == 1]
        self.assertEqual(data[:3], [struct.pack(">BBHI", 1, 0x80, index, 0) for index in (0, 0, 1)])
        self.assertEqual(len(retries), 1)
        bus = DropBus(99)
        with self.assertRaisesRegex(TimeoutError, "bin 0.*6 attempts"):
            CoggingReader(bus, timeout=0.002).download()
        self.assertEqual(len([m for m in bus.sent if m.data[0] == 1]), 6)

    def test_boot_only_lut_and_processing_states_reject_full_download(self):
        bus = FirmwareBus(0, 0x0A)
        with self.assertRaises(RuntimeError):
            CoggingReader(bus).download()
        self.assertEqual(len(bus.sent), 1)
        result = CoggingReader(bus).download(final_only=True)
        self.assertEqual(tuple(result.series), (0x80,))
        for state, flags in ((1, 0x07), (2, 0x07), (3, 0), (0, 0)):
            with self.subTest(state=state, flags=flags):
                with self.assertRaises(RuntimeError):
                    CoggingReader(FirmwareBus(state, flags)).download(final_only=True)

    def test_reset_between_sweeps_invalidates_capture(self):
        class ResetBus(FirmwareBus):
            def send(self, message, timeout):
                if bytes(message.data[:4]) == struct.pack(">BBH", 1, 0, 1439):
                    super().send(message, timeout)
                    self.state, self.flags = 0, 0x0A
                else:
                    super().send(message, timeout)

        bus = ResetBus()
        with self.assertRaisesRegex(RuntimeError, "state changed"):
            CoggingReader(bus).download()
        self.assertEqual(len([m for m in bus.sent if m.data[0] == 1]), BIN_COUNT)

    def test_cancel_and_control_wire_format(self):
        stop, bus = threading.Event(), FirmwareBus()
        reader = CoggingReader(bus, stop_event=stop)
        with self.assertRaises(CoggingCancelled):
            reader.download(on_progress=lambda *_: stop.set())
        self.assertEqual(len(bus.sent), 2)
        reader.abort_measurement()  # Stop is allowed even after cancellation.
        self.assertEqual(bus.sent[-1].arbitration_id, 0x3000)
        self.assertEqual(bytes(bus.sent[-1].data), bytes(4))
        stop.clear()
        reader.start_measurement(5)
        self.assertEqual(bytes(bus.sent[-1].data), bytes.fromhex("000001f4"))
        reader.set_compensation(False)
        reader.set_compensation(True)
        self.assertEqual([(m.arbitration_id, bytes(m.data)) for m in bus.sent[-2:]],
                         [(0x3100, b"\x00"), (0x3100, b"\x01")])
        for rpm in (-1, 0.5, 20.01, math.nan, math.inf):
            with self.assertRaises(ValueError):
                pack_measurement_rpm(rpm)

    def test_seven_headerless_files_preserve_nan_inf_and_reject_partial_or_overwrite(self):
        status = CoggingStatus.decode(status_payload())
        series = {key: (0.0,) * BIN_COUNT for key in SELECTOR_FILES}
        series[0] = (math.nan, math.inf, -math.inf, -0.0) + (0.0,) * (BIN_COUNT - 4)
        capture = CoggingCapture(0, status, series)
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "capture"
            write_cogging_files(target, capture)
            self.assertEqual({p.name for p in target.iterdir()}, set(SELECTOR_FILES.values()))
            for path in target.iterdir():
                self.assertEqual(len(path.read_text().splitlines()), BIN_COUNT)
            values = [float(line) for line in (target / SELECTOR_FILES[0]).read_text().splitlines()]
            self.assertTrue(math.isnan(values[0]))
            self.assertEqual(values[1:3], [math.inf, -math.inf])
            self.assertEqual(math.copysign(1, values[3]), -1)
            with self.assertRaises(FileExistsError):
                write_cogging_files(target, capture)
            partial = CoggingCapture(0, status, {0: (1.0,)})
            with self.assertRaises(ValueError):
                write_cogging_files(Path(folder) / "partial", partial)
            self.assertFalse((Path(folder) / "partial").exists())


class CoggingGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def pump_until(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.002)
        self.app.processEvents()
        self.assertTrue(predicate())

    def window(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        settings = QtCore.QSettings(str(Path(folder.name) / "settings.ini"), QtCore.QSettings.IniFormat)
        with patch("cogging_LUT.QtCore.QSettings", return_value=settings):
            window = CoggingWindow()
        window.interface.setCurrentText("slcan")
        window.channel.setEditText("COM_TEST")
        return window

    def connect(self, window, bus):
        with patch("cogging_LUT.can.Bus", return_value=bus):
            window._connect()
            self.pump_until(lambda: window.connected and window.last_status is not None and not window.job)

    def cleanup(self, window):
        window.close()
        self.pump_until(lambda: window.worker is None)
        window.deleteLater()
        self.app.processEvents()

    def test_ready_download_flash_failure_and_background_save(self):
        window, bus = self.window(), FirmwareBus(flags=0x13)
        try:
            self.connect(window, bus)
            self.assertIn("Flash 저장 실패", window.status_label.text())
            self.assertTrue(window.download_button.isEnabled())
            window._begin_download(False)
            self.pump_until(lambda: window.capture is not None and not window.job)
            self.assertEqual(len(window.capture.series), 7)
            self.assertTrue(window.save_button.isEnabled())
            window._disconnect()
            self.pump_until(lambda: window.worker is None)
            self.assertTrue(window.save_button.isEnabled())
            with tempfile.TemporaryDirectory() as folder:
                target = Path(folder) / "result"
                window._start_save(target)
                self.pump_until(lambda: window.save_worker is None)
                self.assertEqual(len(list(target.iterdir())), 7)
        finally:
            self.cleanup(window)
        self.assertTrue(bus.closed)

    def test_measurement_completion_auto_downloads_and_close_aborts_active_measurement(self):
        class CompletingBus(FirmwareBus):
            def __init__(self):
                super().__init__(0, 0)
                self.active_reads = 0

            def send(self, message, timeout):
                if message.arbitration_id >> 8 == 0x32 and message.data[0] == 0 and self.state == 1:
                    self.active_reads += 1
                    if self.active_reads >= 2:
                        self.state, self.flags = 2, 0x0B
                super().send(message, timeout)

        window, bus = self.window(), CompletingBus()
        try:
            self.connect(window, bus)
            window._start_measurement()
            self.pump_until(lambda: window.capture is not None)
            self.assertEqual(len(window.capture.series), 7)
            self.assertFalse(window.measuring)
        finally:
            self.cleanup(window)
        window, bus = self.window(), FirmwareBus(0, 0)
        try:
            self.connect(window, bus)
            window.auto_download.setChecked(False)
            window._start_measurement()
            self.pump_until(lambda: window.measuring and not window.job)
            self.assertFalse(window.download_button.isEnabled())
            self.assertFalse(window.close())
            self.pump_until(lambda: window.worker is None)
            commands = [m for m in bus.sent if m.arbitration_id >> 8 == 0x30]
            self.assertEqual([struct.unpack(">i", m.data)[0] for m in commands], [500, 0])
        finally:
            self.cleanup(window)

    def test_boot_enables_final_only_and_gui_stays_responsive_during_wait(self):
        window, bus = self.window(), FirmwareBus(0, 0x0A)
        try:
            self.connect(window, bus)
            self.assertFalse(window.download_button.isEnabled())
            self.assertTrue(window.final_button.isEnabled())
            window._begin_download(True)
            self.pump_until(lambda: window.capture is not None)
            self.assertEqual(tuple(window.capture.series), (0x80,))
        finally:
            self.cleanup(window)

        class SilentDataBus(FirmwareBus):
            def send(self, message, timeout):
                if message.data[0] == 1:
                    self.sent.append(message)
                else:
                    super().send(message, timeout)

        window, bus = self.window(), SilentDataBus()
        beats = []
        heartbeat = QtCore.QTimer()
        heartbeat.timeout.connect(lambda: beats.append(time.monotonic()))
        heartbeat.start(5)
        try:
            self.connect(window, bus)
            window._begin_download(False)
            self.pump_until(lambda: window.job == "download")
            beats.clear()
            self.pump_until(lambda: len(beats) >= 5)
            window._cancel_download()
            self.pump_until(lambda: not window.job)
            self.assertIsNone(window.capture)
            self.assertFalse(window.save_button.isEnabled())
        finally:
            heartbeat.stop()
            self.cleanup(window)


if __name__ == "__main__":
    unittest.main()
