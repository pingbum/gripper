"""Offline tests: no physical CAN device is opened or transmitted to."""
import csv
import json
import os
from pathlib import Path
import sys
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import can
from PyQt5 import QtCore, QtWidgets
from can_io import CANBusManager, CANReaderThread, connection_options
from can_workers import CANWriterThread, CaptureWriter, ReferenceCommand
from gui.main_window import MainWindow


def wait_until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.002)
    return predicate()


class FakeBus:
    def __init__(self, delay=0, fail=False):
        self.sent = []
        self.delay = delay
        self.fail = fail
        self.closed = False

    def send(self, message, timeout):
        if self.closed:
            raise RuntimeError("closed")
        if self.fail:
            raise RuntimeError("simulated TX failure")
        time.sleep(self.delay)
        self.sent.append((time.perf_counter(), message))

    def shutdown(self):
        self.closed = True


class WorkerTests(unittest.TestCase):
    def test_pause_discards_work_and_resume_accepts_only_new_commands(self):
        bus = FakeBus()
        worker = CANWriterThread(bus)
        old_generation = worker.transport_generation()
        worker.enqueue(0x8700, bytes(8))
        worker.set_periodic(ReferenceCommand(1, 1, "Sine", 0.1, 1), 10)
        worker.pause()
        self.assertTrue(worker.commands.empty())
        self.assertFalse(worker.periodic_active())
        with self.assertRaisesRegex(RuntimeError, "paused"):
            worker.enqueue(0x8701, bytes(8))
        with self.assertRaisesRegex(RuntimeError, "paused"):
            worker.set_periodic(ReferenceCommand(1, 1, "Sine", 0.1, 1), 10)
        worker.resume(worker.transport_generation())
        self.assertFalse(worker._send(worker._message(0x8700, bytes(8)), 0.02, old_generation))
        worker.start()
        try:
            worker.enqueue(0x8701, bytes(8))
            self.assertTrue(wait_until(lambda: len(bus.sent) == 1))
            self.assertEqual(bus.sent[0][1].arbitration_id, 0x8701)
            self.assertFalse(worker.periodic_active())
        finally:
            worker.stop()
            worker.join(1)

    def test_resume_requires_active_kernel_state_and_no_new_warning(self):
        manager = CANBusManager()
        bus = FakeBus()
        with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
            manager.connect("socketcan", "can0", 1000000)
        manager.writer.pause()

        def state_result(state):
            return SimpleNamespace(stdout=json.dumps([
                {"linkinfo": {"info_data": {"state": state}}}]))

        try:
            for state in ("ERROR-PASSIVE", "BUS-OFF", "ERROR-WARNING", "UNKNOWN"):
                with patch("can_io.subprocess.run", return_value=state_result(state)):
                    with self.assertRaisesRegex(RuntimeError, "remains paused"):
                        manager.resume_tx()
                self.assertTrue(manager.writer.pause_event.is_set())

            def new_warning(*args, **kwargs):
                manager.writer.pause()
                return state_result("ERROR-ACTIVE")

            with patch("can_io.subprocess.run", side_effect=new_warning):
                with self.assertRaisesRegex(RuntimeError, "New CAN warning"):
                    manager.resume_tx()
            self.assertTrue(manager.writer.pause_event.is_set())
            with patch("can_io.subprocess.run", return_value=state_result("ERROR-ACTIVE")) as run:
                manager.resume_tx()
                self.assertEqual(run.call_args.args[0],
                                 ["ip", "-details", "-json", "link", "show", "dev", "can0"])
            self.assertFalse(manager.writer.pause_event.is_set())
            self.assertEqual(bus.sent, [])
        finally:
            manager.disconnect()

    def test_fd_manual_and_periodic_transmission(self):
        for interface, channel in (("socketcan", "can0"), ("slcan", "COM_TEST")):
            with self.subTest(interface=interface):
                bus = FakeBus()
                manager = CANBusManager()
                with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
                    manager.connect(interface, channel, 1000000)
                try:
                    for size in (1, 4, 8):
                        manager.send_ext(0x8701, bytes(size))
                    self.assertTrue(wait_until(lambda: len(bus.sent) >= 3))
                    manager.writer.set_periodic(ReferenceCommand(1, 1, "Sine", 0.1, 1), 10)
                    self.assertTrue(wait_until(lambda: len(bus.sent) >= 4))
                finally:
                    manager.disconnect()
                self.assertEqual([len(msg.data) for _, msg in bus.sent[:3]], [1, 4, 8])
                self.assertEqual(bus.sent[3][1].arbitration_id, 0x101)
                for _, message in bus.sent:
                    self.assertTrue(message.is_extended_id)
                    self.assertTrue(message.is_fd)
                    self.assertTrue(message.bitrate_switch)

    def test_no_brs_manual_and_periodic_remain_fd(self):
        bus = FakeBus()
        manager = CANBusManager()
        with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
            manager.connect("socketcan", "can0", 1000000, tx_brs=False)
        try:
            for size in (1, 4, 8):
                manager.send_ext(0x8700, bytes(size))
            self.assertTrue(wait_until(lambda: len(bus.sent) == 3))
            manager.writer.set_periodic(ReferenceCommand(0, 1, "Sine", 0.1, 1), 10)
            self.assertTrue(wait_until(lambda: len(bus.sent) >= 4))
        finally:
            manager.disconnect()
        self.assertEqual([len(msg.data) for _, msg in bus.sent[:3]], [1, 4, 8])
        for _, message in bus.sent:
            self.assertTrue(message.is_fd and message.is_extended_id)
            self.assertFalse(message.bitrate_switch)

    def test_backend_options_do_not_pass_bitrate_to_socketcan(self):
        with patch("can_io.sys.platform", "linux"):
            options = connection_options("socketcan", "can0", 1000000)
            self.assertTrue(options["fd"])
            self.assertNotIn("bitrate", options)
            self.assertFalse(options["receive_own_messages"])
        with patch("can_io.sys.platform", "win32"):
            with self.assertRaises(ValueError):
                connection_options("socketcan", "can0", 1000000)
        options = connection_options("slcan", "COM11", 1000000)
        self.assertNotIn("tty_baudrate", options)
        self.assertNotIn("bitrate", options)
        self.assertEqual(options["timing"].nom_bitrate, 1000000)
        self.assertEqual(options["timing"].data_bitrate, 5000000)
        self.assertTrue(options["ignore_config"])
        with self.assertRaisesRegex(ValueError, "Classic-only"):
            connection_options("gs_usb", "0", 1000000)
        with self.assertRaisesRegex(ValueError, "1M/5M"):
            connection_options("slcan", "COM11", 500000)

    def test_reference_cap_and_payload(self):
        worker = CANWriterThread(FakeBus())
        for hz in (0, 1001, 2000, float("nan")):
            with self.assertRaises(ValueError):
                worker.set_periodic(lambda t: (0, b"", 0), hz)
        factory = ReferenceCommand(1, 1, "Square", 0.1, 1)
        eid, payload, value = factory(0)
        self.assertEqual((eid, payload, value), (0x101, b"\x00\x00\x00\x64" + bytes(4), 0.1))
        with self.assertRaises((ValueError, OverflowError, struct.error)):
            ReferenceCommand(1, 1, "Sine", 1e9, 1)

    def test_low_reference_rate_does_not_hold_up_manual_commands(self):
        bus = FakeBus()
        worker = CANWriterThread(bus)
        worker.set_periodic(ReferenceCommand(0, 3, "Sine", 10, 1), 1)
        worker.start()
        try:
            self.assertTrue(wait_until(lambda: len(bus.sent) >= 1))
            worker.enqueue(0x8701, bytes(8))
            self.assertTrue(wait_until(lambda: any(msg.arbitration_id == 0x8701
                                                  for _, msg in bus.sent), timeout=0.2))
        finally:
            worker.stop()
            worker.join(1)

    def test_slow_sender_skips_deadlines_and_never_accumulates_reference(self):
        bus = FakeBus(delay=0.008)
        worker = CANWriterThread(bus)
        worker.set_periodic(ReferenceCommand(0, 3, "Sine", 10, 1), 1000)
        worker.start()
        try:
            self.assertTrue(wait_until(lambda: len(bus.sent) >= 5))
            worker.stop_periodic()
            time.sleep(0.02)  # One backend call may already be in flight.
            count = len(bus.sent)
            time.sleep(0.03)
            self.assertEqual(len(bus.sent), count)
            self.assertTrue(worker.commands.empty())
            sent, skipped, errors = worker.take_stats()
            self.assertGreater(skipped, 0)
            self.assertEqual(sent, count)
            self.assertEqual(errors, [])
        finally:
            worker.stop()
            worker.join(1)
        self.assertFalse(worker.is_alive())

    def test_tx_failure_stops_worker_without_replaying_queued_commands(self):
        bus = FakeBus(fail=True)
        worker = CANWriterThread(bus)
        worker.enqueue(1, bytes(8))
        worker.enqueue(2, bytes(8))
        worker.start()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertTrue(worker.stop_event.is_set())
        self.assertIn("simulated TX failure", worker.take_stats()[2][0])
        self.assertEqual(bus.sent, [])
        with self.assertRaises(RuntimeError):
            worker.enqueue(3, bytes(8))

    def test_queue_is_bounded_and_reconnect_has_no_previous_work(self):
        worker = CANWriterThread(FakeBus())
        for i in range(worker.commands.maxsize):
            worker.enqueue(i, b"\x00")
        with self.assertRaises(RuntimeError):
            worker.enqueue(255, b"\x00")
        manager = CANBusManager()
        manager.bus, manager.writer = worker.bus, worker
        worker.start()
        manager.disconnect()
        replacement = FakeBus()
        with patch("can_io.can.Bus", return_value=replacement):
            manager.connect("slcan", "COM_TEST", 1000000)
        time.sleep(0.02)
        self.assertEqual(replacement.sent, [])
        self.assertFalse(manager.writer.periodic_active())
        manager.disconnect()

    def test_csv_keeps_both_ids_before_display_reduction_and_flushes(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "rx.csv"
            capture = CaptureWriter(path)
            capture.start()
            stop = threading.Event()
            messages = iter(can.Message(arbitration_id=i % 2, is_extended_id=True,
                                        timestamp=i / 2000, data=bytes(8)) for i in range(4000))

            class InputBus:
                def recv(self, timeout):
                    result = next(messages, None)
                    if result is None:
                        stop.set()
                    return result

            reader = CANReaderThread(InputBus(), stop)
            reader.recorder = capture
            reader.run()
            capture.stop()
            capture.join(3)
            self.assertFalse(capture.is_alive())
            self.assertEqual(capture.snapshot(), (4000, 0, None))
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 4000)
            self.assertEqual({r["arbitration_id"] for r in rows}, {"00000000", "00000001"})
            self.assertIsNotNone(reader.take_latest_status(0))
            self.assertIsNotNone(reader.take_latest_status(1))

    def test_csv_overflow_reports_drops_and_existing_files_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "rx.csv"
            capture = CaptureWriter(path, capacity=2)
            message = can.Message(arbitration_id=1, data=bytes(8))
            for _ in range(5):
                capture.offer(message)
            capture.stop()
            capture.start()
            capture.join(1)
            self.assertEqual(capture.snapshot(), (2, 3, None))
            before = path.read_bytes()
            duplicate = CaptureWriter(path)
            duplicate.start()
            duplicate.join(1)
            self.assertIsNotNone(duplicate.error)
            self.assertEqual(path.read_bytes(), before)


class GuiWorkerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def setUp(self):
        values = {}

        def value(key, default=None, type=None):
            result = values.get(key, default)
            return type(result) if type is not None else result

        settings = SimpleNamespace(value=value, setValue=values.__setitem__)
        settings_patch = patch.object(MainWindow, "_connection_settings", return_value=settings)
        settings_patch.start()
        self.addCleanup(settings_patch.stop)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.app.processEvents()
            time.sleep(0.001)

    def test_connect_discovers_broadcasts_without_any_automatic_transmission(self):
        class BroadcastBus(FakeBus):
            def __init__(self):
                super().__init__()
                self.next_id = 0

            def recv(self, timeout):
                time.sleep(0.01)
                self.next_id ^= 1
                return can.Message(arbitration_id=self.next_id, is_extended_id=True,
                                   is_fd=True, bitrate_switch=True, data=bytes(8))

        window = MainWindow()
        bus = BroadcastBus()
        try:
            window.iface_combo.setCurrentText("socketcan")
            window.channel_edit.setText("can0")
            self.assertFalse(window.cb_tx_brs.isChecked())
            with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
                window._on_connect()
            # Cover the old 300-ms automatic parameter-read callback too.
            self.pump(0.55)
            self.assertEqual(bus.sent, [])
            self.assertEqual(window.scan_hits, {0, 1})
            self.assertEqual({window.combo_motor.itemData(i)
                              for i in range(1, window.combo_motor.count())}, {0, 1})
            self.assertGreater(len(window.t_buf), 0)
            self.assertFalse(window.read_pending)
            window.btn_ctrl_read.click()
            self.assertTrue(wait_until(lambda: len(bus.sent) > 0))
            msg = bus.sent[0][1]
            self.assertEqual(msg.arbitration_id, 0x9100 | window.spin_driver.value())
            self.assertTrue(msg.is_fd)
            self.assertFalse(msg.bitrate_switch)
        finally:
            window._on_disconnect()
            self.pump(0.1)
            window.close()
            window.deleteLater()

    def test_tx_error_passive_keeps_gui_receiving_and_requires_explicit_resume(self):
        class PassiveBus(FakeBus):
            warning_sent = False

            def recv(self, timeout):
                time.sleep(0.01)
                if not self.warning_sent:
                    self.warning_sent = True
                    return can.Message(arbitration_id=0x04, is_error_frame=True,
                                       data=b"\x00\x20" + bytes(6))
                return can.Message(arbitration_id=1, is_extended_id=True,
                                   is_fd=True, bitrate_switch=True, data=bytes(8))

        window = MainWindow()
        bus = PassiveBus()
        try:
            window.iface_combo.setCurrentText("socketcan")
            window.channel_edit.setText("can0")
            with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
                window._on_connect()
            self.pump(0.2)
            window._update_rate_label(force=True)
            self.assertIs(window.manager.bus, bus)
            self.assertIsNone(window._disconnect_thread)
            self.assertIsNone(window.reader_thread.last_error)
            self.assertTrue(window.manager.writer.pause_event.is_set())
            self.assertFalse(window.manager.writer.stop_event.is_set())
            self.assertGreater(len(window.t_buf), 0)
            self.assertIn("RX continues", window.lbl_tx_health.text())
            self.assertEqual(window.log.toPlainText().count("CAN warning:"), 1)
            self.assertNotIn("Disconnected", window.log.toPlainText())
            self.assertEqual(bus.sent, [])
            self.assertTrue(window.btn_resume_tx.isEnabled())
            result = SimpleNamespace(stdout=json.dumps([
                {"linkinfo": {"info_data": {"state": "ERROR-ACTIVE"}}}]))
            with patch("can_io.subprocess.run", return_value=result):
                window.btn_resume_tx.click()
                self.pump(0.1)
            self.assertFalse(window.manager.writer.pause_event.is_set())
            self.assertFalse(window.btn_resume_tx.isEnabled())
            self.assertIn("TX ready for new requests", window.log.toPlainText())
            self.assertEqual(bus.sent, [])
            window.btn_ctrl_read.click()
            self.assertTrue(wait_until(lambda: len(bus.sent) == 1))
            self.assertTrue(bus.sent[0][1].is_fd)
            self.assertFalse(bus.sent[0][1].bitrate_switch)
        finally:
            window._on_disconnect()
            self.pump(0.1)
            window.close()
            window.deleteLater()

    def test_repeated_rx_passive_warnings_allow_explicit_fd_read(self):
        class RxPassiveBus(FakeBus):
            count = 0

            def recv(self, timeout):
                time.sleep(0.01)
                self.count += 1
                if self.count % 2:
                    return can.Message(arbitration_id=0x04, is_error_frame=True,
                                       data=b"\x00\x10" + bytes(5) + bytes([0x79 + self.count % 2]))
                return can.Message(arbitration_id=0, is_extended_id=True,
                                   is_fd=True, bitrate_switch=True, data=bytes(8))

        window = MainWindow()
        bus = RxPassiveBus()
        try:
            window.spin_listen.setValue(0)
            window.iface_combo.setCurrentText("socketcan")
            window.channel_edit.setText("can0")
            with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
                window._on_connect()
            self.pump(0.2)
            window._update_rate_label(force=True)
            self.assertIs(window.manager.bus, bus)
            self.assertFalse(window.manager.writer.pause_event.is_set())
            self.assertFalse(window.manager.writer.stop_event.is_set())
            self.assertFalse(window.btn_resume_tx.isEnabled())
            self.assertGreater(window.reader_thread.warning_count, 1)
            self.assertEqual(window.log.toPlainText().count("CAN warning:"), 1)
            self.assertIn("TX ready", window.lbl_tx_health.text())
            self.assertGreater(len(window.t_buf), 0)
            self.assertEqual(bus.sent, [])
            window.btn_ctrl_read.click()
            self.assertTrue(wait_until(lambda: len(bus.sent) == 1))
            self.assertTrue(bus.sent[0][1].is_fd)
            self.assertFalse(bus.sent[0][1].bitrate_switch)
            self.pump(0.1)
            self.assertFalse(window.manager.writer.pause_event.is_set())
        finally:
            window._on_disconnect()
            self.pump(0.1)
            window.close()
            window.deleteLater()

    def test_tx_brs_choice_applies_to_read_and_calibration_while_fd_brs_rx_continues(self):
        class BroadcastBus(FakeBus):
            def recv(self, timeout):
                time.sleep(0.01)
                return can.Message(arbitration_id=1, is_extended_id=True,
                                   is_fd=True, bitrate_switch=True, data=bytes(8))

        for brs in (False, True):
            with self.subTest(brs=brs):
                window = MainWindow()
                bus = BroadcastBus()
                try:
                    window.iface_combo.setCurrentText("socketcan")
                    window.channel_edit.setText("can0")
                    window.cb_tx_brs.setChecked(brs)
                    with patch("can_io.sys.platform", "linux"), patch("can_io.can.Bus", return_value=bus):
                        window._on_connect()
                    self.pump(0.1)
                    self.assertFalse(window.cb_tx_brs.isEnabled())
                    self.assertGreater(len(window.t_buf), 0)
                    self.assertEqual(bus.sent, [])
                    window.btn_ctrl_read.click()
                    window.btn_calib.click()
                    self.assertTrue(wait_until(lambda: len(bus.sent) == 2))
                    self.assertEqual([msg.arbitration_id for _, msg in bus.sent], [0x9101, 0x0601])
                    for _, msg in bus.sent:
                        self.assertTrue(msg.is_fd and msg.is_extended_id)
                        self.assertEqual(msg.bitrate_switch, brs)
                    window._on_disconnect()
                    self.pump(0.1)
                    self.assertTrue(window.cb_tx_brs.isEnabled())
                    window.iface_combo.setCurrentText("slcan")
                    window.iface_combo.setCurrentText("socketcan")
                    self.assertEqual(window.cb_tx_brs.isChecked(), brs)
                finally:
                    window._on_disconnect()
                    self.pump(0.1)
                    window.close()
                    window.deleteLater()

    def test_gui_runs_while_backend_send_is_blocked_and_disconnect_is_async(self):
        entered, release = threading.Event(), threading.Event()

        class BlockingBus(FakeBus):
            def send(self, message, timeout):
                entered.set()
                release.wait(1)
                super().send(message, timeout)

        window = MainWindow()
        bus = BlockingBus()
        with patch("can_io.can.Bus", return_value=bus):
            window.manager.connect("slcan", "COM_TEST", 1000000)
        beats = []
        heartbeat = QtCore.QTimer()
        heartbeat.timeout.connect(lambda: beats.append(time.monotonic()))
        heartbeat.start(10)
        try:
            window._on_ref_toggle()
            self.assertTrue(entered.wait(1))
            self.pump(0.12)
            self.assertGreaterEqual(len(beats), 5)
            self.assertEqual(window.timer.interval(), 25)
            window._on_disconnect()
            self.assertFalse(window.ref_running)
            self.assertFalse(window.btn_connect.isEnabled())
            release.set()
            self.pump(0.1)
            self.assertIsNone(window.manager.bus)
            self.assertIsNone(window._disconnect_thread)
        finally:
            release.set()
            heartbeat.stop()
            window._on_disconnect()
            self.pump(0.1)
            window.close()
            window.deleteLater()

    def test_socketcan_and_slcan_ui_fields(self):
        window = MainWindow()
        try:
            window.iface_combo.setCurrentText("socketcan")
            self.assertFalse(window.bitrate_edit.isEnabled())
            window.iface_combo.setCurrentText("slcan")
            self.assertTrue(window.bitrate_edit.isEnabled())
            self.assertEqual(window.spin_tx_hz.maximum(), 1000)
        finally:
            window.close()
            window.deleteLater()

    def test_2000hz_rx_and_1000hz_target_tx_with_rendering_and_recording(self):
        class TimedBus(FakeBus):
            def __init__(self):
                super().__init__()
                self.deadline = time.perf_counter()
                self.received = 0

            def recv(self, timeout):
                time.sleep(max(0, self.deadline - time.perf_counter()))
                self.deadline += 0.0005
                self.received += 1
                return can.Message(arbitration_id=self.received % 2,
                                   is_extended_id=True, timestamp=time.time(),
                                   data=struct.pack('>hhhbb', self.received % 3600, 100, 10, 0, 0))

        with tempfile.TemporaryDirectory() as folder:
            window = MainWindow()
            bus = TimedBus()
            with patch("can_io.can.Bus", return_value=bus):
                window.manager.connect("slcan", "COM_TEST", 1000000)
            capture = CaptureWriter(Path(folder) / "load.csv")
            window.capture = capture
            window.reader_thread = CANReaderThread(bus, window.reader_stop)
            window.reader_thread.recorder = capture
            capture.start()
            window.reader_thread.start()
            window.show()
            try:
                window._on_ref_toggle()
                self.pump(1.1)
                self.assertGreater(bus.received, 1000)
                self.assertGreater(len(bus.sent), 50)
                self.assertLess(len(window.t_buf), 60)
                self.assertGreater(len(window.t_buf), 5)
                window.reader_stop.set()
                window.reader_thread.join(1)
                capture.stop()
                capture.join(2)
                self.assertEqual(capture.snapshot(), (bus.received, 0, None))
            finally:
                window._on_disconnect()
                self.pump(0.15)
                window.close()
                window.deleteLater()


if __name__ == "__main__":
    unittest.main()
