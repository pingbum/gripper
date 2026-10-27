"""Open-loop wire format, stop ordering and GUI lifecycle; no hardware TX."""
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

from PyQt5 import QtCore, QtWidgets
from can_workers import CANWriterThread, ReferenceCommand
from gui.main_window import MainWindow
from open_loop import OpenLoopBusManager, OpenLoopWindow, OpenLoopWriter, pack_open_loop_command


class Bus:
    def __init__(self):
        self.messages = []
        self.fail = False
        self.closed = False
        self.received = queue.Queue()

    def send(self, message, timeout):
        if self.fail:
            raise RuntimeError("TX unavailable")
        self.messages.append(message)

    def shutdown(self):
        self.closed = True

    def recv(self, timeout):
        try:
            return self.received.get(timeout=timeout)
        except queue.Empty:
            return None


def manager_with_bus():
    manager = OpenLoopBusManager()
    manager.bus = Bus()
    manager.writer = OpenLoopWriter(manager.bus)
    return manager


class OpenLoopTests(unittest.TestCase):
    def test_wire_examples_and_limits(self):
        for rpm, voltage, expected in (
            (55, 2, "00000037000007d0"),
            (-55, 2, "ffffffc9000007d0"),
            (100, .5, "00000064000001f4"),
            (0, 0, "0000000000000000"),
            (600, 2.24, "00000258000008c0"),
        ):
            self.assertEqual(pack_open_loop_command(rpm, voltage).hex(), expected)
        for rpm, voltage in ((601, 0), (-601, 0), (0, -.001), (0, 2.241),
                             (float("nan"), 0), (0, float("inf"))):
            with self.assertRaises(ValueError):
                pack_open_loop_command(rpm, voltage)

    def test_brs_off_and_immediate_stop_discards_stale_commands(self):
        manager = manager_with_bus()
        manager.send_open_loop(7, -55, 2)
        message, timeout, generation = manager.writer.commands.get_nowait()
        self.assertEqual(message.arbitration_id, 0x407)
        self.assertTrue(message.is_fd and message.is_extended_id)
        self.assertFalse(message.bitrate_switch)
        self.assertTrue(CANWriterThread(manager.bus, bitrate_switch=True)
                        ._message(0x107, bytes(8)).bitrate_switch)
        manager.send_open_loop(7, 100, .5)
        manager.writer.set_periodic(ReferenceCommand(7, 1, "Sine", .1, 1), 10)
        manager.stop_open_loop(7)
        self.assertFalse(manager.writer._send(message, timeout, generation))
        self.assertTrue(manager.writer.commands.empty())
        self.assertFalse(manager.writer.periodic_active())
        self.assertEqual(len(manager.bus.messages), 1)
        stop = manager.bus.messages[0]
        self.assertEqual(bytes(stop.data), bytes(8))
        self.assertFalse(stop.bitrate_switch)
        self.assertEqual(manager.open_loop_motors, set())

    def test_zero_speed_is_not_stop_and_shutdown_tracks_multiple_ids(self):
        manager = manager_with_bus()
        manager.send_open_loop(0, 0, 2)
        manager.send_open_loop(1, -600, 2.24)
        self.assertEqual(manager.open_loop_motors, {0, 1})
        manager.writer.pause()
        manager.bus.fail = True
        with self.assertRaises(RuntimeError):
            manager.stop_open_loop_motors()
        self.assertEqual(manager.open_loop_motors, {0, 1})
        manager.bus.fail = False
        manager.stop_open_loop_motors()
        self.assertEqual([m.arbitration_id for m in manager.bus.messages], [0x400, 0x401])
        self.assertTrue(all(bytes(m.data) == bytes(8) for m in manager.bus.messages))


class OpenLoopGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def pump_until(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.002)
        self.app.processEvents()
        self.assertTrue(predicate())

    def window(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        settings = QtCore.QSettings(str(Path(folder.name) / "settings.ini"),
                                    QtCore.QSettings.IniFormat)
        with patch("open_loop.QtCore.QSettings", return_value=settings):
            window = OpenLoopWindow()
        window.interface.setCurrentText("slcan")
        window.channel.setEditText("COM_TEST")
        return window

    def cleanup(self, window):
        window.close()
        self.pump_until(lambda: window._action_thread is None and window.manager.bus is None)
        window.deleteLater()
        self.app.processEvents()

    def test_gui_start_update_stop_and_disconnect(self):
        window = self.window()
        bus = Bus()
        try:
            with patch("open_loop.can.Bus", return_value=bus):
                window._on_connect()
                self.pump_until(lambda: window._action_thread is None)
            self.assertEqual(bus.messages, [])  # Connect is RX only.
            window.motor_id.setValue(0)
            window.electrical_rpm.setValue(55)
            window.voltage.setValue(2)
            window._on_start()
            self.pump_until(lambda: len(bus.messages) == 1)
            self.assertEqual(struct.unpack(">ii", bus.messages[0].data), (55, 2000))
            window.electrical_rpm.setValue(-70)
            self.assertEqual(len(bus.messages), 1)  # Editing never transmits.
            self.assertIn("-10.00", window.mechanical_rpm.text())
            self.assertIn("7 pole pairs", window.mechanical_rpm.text())
            window._on_start()
            self.pump_until(lambda: len(bus.messages) == 2)
            window._on_stop()
            self.pump_until(lambda: window._action_thread is None)
            self.assertEqual(bytes(bus.messages[-1].data), bytes(8))
            window._on_start()
            self.pump_until(lambda: len(bus.messages) == 4)
            window.motor_id.setValue(1)
            window._on_start()
            self.pump_until(lambda: len(bus.messages) == 5)
            self.assertFalse(window.close())
            self.pump_until(lambda: window._action_thread is None and window.manager.bus is None)
            self.assertEqual([m.arbitration_id for m in bus.messages[-2:]], [0x400, 0x401])
            self.assertTrue(all(bytes(m.data) == bytes(8) for m in bus.messages[-2:]))
            self.assertTrue(all(m.is_fd and m.is_extended_id and not m.bitrate_switch
                                for m in bus.messages))
            self.assertTrue(bus.closed)
        finally:
            self.cleanup(window)

    def test_stop_does_not_block_gui_even_when_backend_send_blocks(self):
        entered, release = threading.Event(), threading.Event()

        class BlockingStopBus(Bus):
            def send(self, message, timeout):
                if bytes(message.data) == bytes(8):
                    entered.set()
                    release.wait(2)
                super().send(message, timeout)

        window, bus = self.window(), BlockingStopBus()
        beats = []
        heartbeat = QtCore.QTimer()
        heartbeat.timeout.connect(lambda: beats.append(time.monotonic()))
        heartbeat.start(5)
        try:
            with patch("open_loop.can.Bus", return_value=bus):
                window._on_connect()
                self.pump_until(lambda: window._action_thread is None)
            window._on_start()
            self.pump_until(lambda: len(bus.messages) == 1)
            beats.clear()
            window._on_stop()
            self.pump_until(entered.is_set)
            self.pump_until(lambda: len(beats) >= 5)
            self.assertFalse(window.start_button.isEnabled())
            release.set()
            self.pump_until(lambda: window._action_thread is None)
            self.assertEqual(bytes(bus.messages[-1].data), bytes(8))
        finally:
            release.set()
            heartbeat.stop()
            self.cleanup(window)

    def test_close_during_slow_connect_cleans_up_without_sending_commands(self):
        entered, release = threading.Event(), threading.Event()
        window, bus = self.window(), Bus()

        def connect(**options):
            entered.set()
            release.wait(2)
            return bus

        try:
            with patch("open_loop.can.Bus", side_effect=connect):
                window._on_connect()
                self.pump_until(entered.is_set)
                self.assertFalse(window.close())
                release.set()
                self.pump_until(lambda: window._action_thread is None and bus.closed)
            self.assertEqual(bus.messages, [])
            self.assertIsNone(window.manager.bus)
        finally:
            release.set()
            self.cleanup(window)

    def test_zero_voltage_start_is_an_explicit_stop(self):
        window, bus = self.window(), Bus()
        try:
            with patch("open_loop.can.Bus", return_value=bus):
                window._on_connect()
                self.pump_until(lambda: window._action_thread is None)
            window.voltage.setValue(0)
            window._on_start()
            self.pump_until(lambda: window._action_thread is None)
            self.assertEqual(len(bus.messages), 1)
            self.assertEqual(bytes(bus.messages[0].data), bytes(8))
        finally:
            self.cleanup(window)

    def test_original_monitor_has_no_open_loop_controls_or_handlers(self):
        window = MainWindow()
        try:
            self.assertFalse(hasattr(window, "spin_open_rpm"))
            self.assertFalse(hasattr(window, "_on_open_loop_start"))
            self.assertFalse(hasattr(window.manager, "send_open_loop"))
        finally:
            window.close()
            window.deleteLater()
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
