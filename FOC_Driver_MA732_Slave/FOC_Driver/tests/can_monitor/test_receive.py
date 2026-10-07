"""Run with: QT_QPA_PLATFORM=offscreen .venv/bin/python tests/can_monitor/test_receive.py"""
import struct
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import can
from PyQt5 import QtWidgets
from can_io import CANReaderThread
from gui.main_window import MainWindow


def status(driver_id, position):
    return can.Message(
        arbitration_id=driver_id,
        is_extended_id=True,
        data=struct.pack(">hhhbb", position * 10, 0, 0, 0, 0),
    )


class MessageBus:
    """Deterministic input; no CAN hardware or transmissions."""
    def __init__(self, messages, stop):
        self.messages = iter(messages)
        self.stop = stop

    def recv(self, timeout):
        message = next(self.messages, None)
        if message is None:
            self.stop.set()
        return message


def receive(messages, **kwargs):
    stop = threading.Event()
    reader = CANReaderThread(MessageBus(messages, stop), stop, **kwargs)
    reader.run()
    return reader


class ReceiveTests(unittest.TestCase):
    def test_fd_broadcasts_discover_motors_without_scan_requests(self):
        scan_hits = set()
        frames = [status(0, 15), status(1, 30)]
        for frame in frames:
            frame.is_fd = frame.bitrate_switch = True
        reader = receive(frames, scan_hits=scan_hits, scan_lock=threading.Lock())
        self.assertEqual(scan_hits, {0, 1})
        self.assertIsNotNone(reader.take_latest_status(0))
        self.assertIsNotNone(reader.take_latest_status(1))

    def test_bus_off_stops_transmission_and_reception(self):
        tx_stop = threading.Event()
        frame = can.Message(arbitration_id=0x40, is_error_frame=True, data=bytes(8))
        reader = receive([frame, status(0, 15)], on_error=tx_stop.set)
        self.assertTrue(tx_stop.is_set())
        self.assertIn("BUS-OFF", reader.last_error)
        self.assertIsNone(reader.take_latest_status(0))

    def test_only_tx_error_passive_pauses_transmission_and_rx_continues(self):
        for flags, side in ((0x10, "RX"), (0x20, "TX"), (0x30, "RX/TX")):
            with self.subTest(side=side):
                tx_stop = threading.Event()
                frame = can.Message(arbitration_id=0x04, is_error_frame=True,
                                    data=bytes([0, flags]) + bytes(6))
                fatal_stop = threading.Event()
                reader = receive([frame, status(0, 15), status(1, 30)],
                                 on_error=fatal_stop.set, on_warning=tx_stop.set)
                self.assertEqual(tx_stop.is_set(), bool(flags & 0x20))
                self.assertFalse(fatal_stop.is_set())
                self.assertIsNone(reader.last_error)
                self.assertIn(f"ERROR-PASSIVE ({side},", reader.last_warning)
                self.assertIn("TX paused" if flags & 0x20 else "TX remains available",
                              reader.last_warning)
                self.assertIsNotNone(reader.take_latest_status(0))
                self.assertIsNotNone(reader.take_latest_status(1))
                self.assertEqual(reader.get_and_reset_rx_count(), 2)

    def test_counter_changes_do_not_repeat_same_warning_and_recovery_is_reported(self):
        passive = [can.Message(arbitration_id=0x04, is_error_frame=True,
                               data=b"\x00\x10" + bytes(5) + bytes([counter]))
                   for counter in (0x79, 0x7a, 0x79)]
        reader = receive(passive)
        self.assertEqual(reader.warning_count, 3)
        notices = reader.take_notices()
        self.assertEqual(len(notices), 1)
        self.assertIn("ERROR-PASSIVE", notices[0][1])
        self.assertEqual(reader.take_notices(), [])
        active = can.Message(arbitration_id=0x04, is_error_frame=True,
                             data=b"\x00\x40" + bytes(6))
        reader = receive(passive + [active, status(0, 15)])
        self.assertIsNone(reader.last_warning)
        self.assertEqual([kind for kind, _ in reader.take_notices()], ["warning", "state"])
        self.assertIsNotNone(reader.take_latest_status(0))

    def test_both_arrival_orders_keep_each_drivers_latest_status(self):
        for order in ((0, 1), (1, 0)):
            with self.subTest(order=order):
                messages = [status(driver, sample + driver)
                            for sample in range(100) for driver in order]
                reader = receive(messages)
                for driver in (0, 1):
                    self.assertEqual(reader.take_latest_status(driver)[1],
                                     bytes(status(driver, 99 + driver).data))
                self.assertEqual(reader.get_and_reset_rx_count(), 200)
                self.assertEqual(reader.get_and_reset_rx_count(), 0)

    def test_second_board_cannot_evict_first_board(self):
        for first in (0, 1):
            with self.subTest(first=first):
                second = 1 - first
                reader = receive([status(first, 15)] + [status(second, 30)] * 10000)
                self.assertEqual(reader.take_latest_status(first)[1], bytes(status(first, 15).data))
                self.assertEqual(reader.take_latest_status(second)[1], bytes(status(second, 30).data))

    def test_consumed_status_is_not_repeated_without_new_data(self):
        reader = receive([status(0, 15), status(1, 30)])
        self.assertIsNotNone(reader.take_latest_status(0))
        self.assertIsNone(reader.take_latest_status(0))
        self.assertIsNotNone(reader.take_latest_status(1))
        self.assertIsNone(reader.take_latest_status(2))

    def test_commands_responses_and_invalid_frames_do_not_replace_status(self):
        scan_hits = set()
        response = b"\x3f\x80\x00\x00"
        read_watch = {"eid": 0x9100}
        messages = [
            status(0, 15),
            can.Message(arbitration_id=0x100, is_extended_id=True, data=bytes(8)),
            can.Message(arbitration_id=0x8700, is_extended_id=True, data=b"\x00"),
            can.Message(arbitration_id=0x9100, is_extended_id=True, data=response),
            can.Message(arbitration_id=0, is_extended_id=False, data=bytes(8)),
            can.Message(arbitration_id=0, is_extended_id=True, data=bytes(8), is_error_frame=True),
            can.Message(arbitration_id=0, is_extended_id=True, is_remote_frame=True, dlc=8),
            can.Message(arbitration_id=0, is_extended_id=True, data=bytes(4)),
            can.Message(arbitration_id=0x10000, is_extended_id=True, data=bytes(8)),
        ]
        reader = receive(messages, scan_hits=scan_hits, scan_lock=threading.Lock(),
                         scan_func_id=0x87, read_watch=read_watch, read_lock=threading.Lock())
        self.assertEqual(reader.take_latest_status(0)[1], bytes(status(0, 15).data))
        self.assertEqual(scan_hits, {0})
        self.assertEqual(read_watch["data"], response)

    def test_new_connection_starts_without_old_samples(self):
        old = receive([status(0, 15)])
        new = receive([])
        self.assertIsNotNone(old.take_latest_status(0))
        self.assertIsNone(new.take_latest_status(0))


class MonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_gui_updates_either_selected_id_from_the_same_received_burst(self):
        for order in ((0, 1), (1, 0)):
            with self.subTest(order=order):
                window = MainWindow()
                window.timer.stop()
                window.scan_timer.stop()
                window.reader_thread = receive([status(driver, 10 + driver) for driver in order])
                for driver in order:
                    window.spin_listen.setValue(driver)
                    window._drain_and_update()
                    self.assertEqual(window.pos_buf[-1], 10 + driver)
                count = len(window.pos_buf)
                window._drain_and_update()
                self.assertEqual(len(window.pos_buf), count)
                window.close()
                window.deleteLater()


if __name__ == "__main__":
    unittest.main()
