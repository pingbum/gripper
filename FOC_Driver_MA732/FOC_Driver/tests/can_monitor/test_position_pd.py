"""Position PD protocol/read-back checks with a fake firmware, no hardware."""
import math
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import can
from PyQt5 import QtWidgets
from can_io import CANReaderThread
from gui.main_window import MainWindow
from protocol import pack_position_gain


class FirmwareBus:
    def __init__(self):
        self.values = {0x11: 100.0, 0x20: 1.0, 0x21: 2.0, 0x22: 0.05, 0x23: 0.003}
        self.sent = []
        self.pending = queue.Queue()

    def send(self, message, timeout):
        self.sent.append(message)
        mode = message.arbitration_id >> 8
        if mode & 0x80:
            self.pending.put(can.Message(arbitration_id=message.arbitration_id,
                                         is_extended_id=True, is_fd=True, bitrate_switch=True,
                                         data=struct.pack(">f", self.values[mode & 0x7F])))
        elif mode in self.values:
            self.values[mode], = struct.unpack(">f", message.data)

    def recv(self, timeout):
        try:
            return self.pending.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        pass


class PositionProtocolTests(unittest.TestCase):
    def test_unscaled_default_zero_and_invalid_gain_payloads(self):
        self.assertEqual(pack_position_gain(0.05).hex(), "3d4ccccd")
        self.assertEqual(pack_position_gain(0.003).hex(), "3b449ba6")
        self.assertEqual(pack_position_gain(0), bytes(4))
        for value in (-0.1, math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                pack_position_gain(value)
        with self.assertRaises(OverflowError):
            pack_position_gain(1e40)

    def test_position_replies_require_fd_brs_on_and_exactly_four_bytes(self):
        for valid in (False, True):
            stop = threading.Event()
            watch = {"eid": 0xA200}
            payload = struct.pack(">f", 0.125)
            messages = [
                can.Message(arbitration_id=0xA200, is_extended_id=True, is_fd=True,
                            bitrate_switch=True, data=payload, is_rx=False),
                can.Message(arbitration_id=0xA200, is_extended_id=True, data=payload),
                can.Message(arbitration_id=0xA200, is_extended_id=True, is_fd=True, data=payload),
                can.Message(arbitration_id=0xA200, is_extended_id=True, is_fd=True,
                            bitrate_switch=True, data=bytes(8)),
                can.Message(arbitration_id=0xA201, is_extended_id=True, is_fd=True,
                            bitrate_switch=True, data=payload),
            ]
            if valid:
                messages.append(can.Message(arbitration_id=0xA200, is_extended_id=True,
                                            is_fd=True, bitrate_switch=True, data=payload))
            stream = iter(messages)

            class InputBus:
                def recv(self, timeout):
                    message = next(stream, None)
                    if message is None:
                        stop.set()
                    return message

            reader = CANReaderThread(InputBus(), stop, read_watch=watch, read_lock=threading.Lock())
            reader.run()
            self.assertEqual(watch.get("data"), payload if valid else None)


class PositionGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.window, self.bus = MainWindow(), FirmwareBus()
        self.window.timer.stop()
        self.window.spin_driver.setValue(0)
        with patch("can_io.can.Bus", return_value=self.bus):
            self.window.manager.connect("slcan", "COM_TEST", 1000000, tx_brs=False)
        self.window.reader_thread = CANReaderThread(self.bus, self.window.reader_stop,
                                                   read_watch=self.window.read_watch,
                                                   read_lock=self.window.read_lock)
        self.window.reader_thread.start()

    def tearDown(self):
        self.window._on_disconnect()
        if self.window._disconnect_thread:
            self.window._disconnect_thread.join(2)
        self.window._poll_background()
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()

    def pump_until(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            self.window._poll_read_response()
            self.app.processEvents()
            time.sleep(0.002)
        self.assertTrue(predicate())

    def test_send_preserves_velocity_commands_and_reads_back_without_flash_or_motion(self):
        self.window.spin_ctrl_kp.setValue(1)
        self.window.spin_ctrl_ki.setValue(2)
        self.window._on_ctrl_send()
        self.pump_until(lambda: len(self.bus.sent) >= 10 and not self.window.read_pending)
        self.assertEqual([m.arbitration_id for m in self.bus.sent],
                         [0x1100, 0x2000, 0x2100, 0x2200, 0x2300,
                          0x9100, 0xA000, 0xA100, 0xA200, 0xA300])
        self.assertEqual(bytes(self.bus.sent[3].data).hex(), "3d4ccccd")
        self.assertEqual(bytes(self.bus.sent[4].data).hex(), "3b449ba6")
        self.assertTrue(all(m.is_fd and m.is_extended_id and not m.bitrate_switch
                            for m in self.bus.sent))
        self.assertAlmostEqual(self.window.spin_ctrl_pos_kp.value(), 0.05)
        self.assertAlmostEqual(self.window.spin_ctrl_pos_kd.value(), 0.003)
        self.assertIn("Position Kd read back", self.window.log.toPlainText())
        self.window._on_flash_update()
        self.pump_until(lambda: len(self.bus.sent) == 11)
        self.assertEqual(self.bus.sent[-1].arbitration_id, 0x1000)

    def test_zero_gains_round_trip_and_invalid_input_submits_no_writes(self):
        for widget in (self.window.spin_ctrl_pos_kp, self.window.spin_ctrl_pos_kd):
            widget.setValue(0)
        self.window._on_ctrl_send()
        self.pump_until(lambda: len(self.bus.sent) >= 10 and not self.window.read_pending)
        self.assertEqual(self.bus.values[0x22], 0)
        self.assertEqual(self.bus.values[0x23], 0)
        count = len(self.bus.sent)
        with patch.object(self.window.spin_ctrl_pos_kd, "value", return_value=math.nan):
            self.window._on_ctrl_send()
        self.assertEqual(len(self.bus.sent), count)

    def test_invalid_read_does_not_change_gain_and_driver_change_cancels_read(self):
        original = self.window.spin_ctrl_pos_kp.value()
        for data in (struct.pack(">f", math.nan), struct.pack(">f", -1), bytes(8)):
            self.assertFalse(self.window._apply_param_read(0x22, data))
            self.assertEqual(self.window.spin_ctrl_pos_kp.value(), original)
        self.window._start_param_read()
        self.window.spin_driver.setValue(1)
        self.window._poll_read_response()
        self.assertFalse(self.window.read_pending)
        self.assertEqual(self.window.param_read_queue, [])
        self.assertIn("Driver ID 변경", self.window.log.toPlainText())


if __name__ == "__main__":
    unittest.main()
