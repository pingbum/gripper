"""Offline LUT protocol, CSV and GUI checks; never opens a physical adapter."""
import csv
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

from LUT import LUTWindow
from lut_reader import LUTCancelled, LUTCapture, LUTMetadata, LUTReader, write_lut_csv


def reply(eid, index, value, **flags):
    options = dict(arbitration_id=eid, data=struct.pack(">If", index, value),
                   is_extended_id=True, is_fd=True, bitrate_switch=False)
    options.update(flags)
    return can.Message(**options)


class FirmwareBus:
    """Respond after send(), keeping the transaction order observable."""
    def __init__(self, offsets=(0.0, 1.0), direction=-1.0):
        self.offsets = offsets
        self.direction = direction
        self.sent = []
        self.pending = queue.Queue()
        self.closed = False

    def send(self, message, timeout):
        if self.closed:
            raise RuntimeError("closed")
        self.sent.append(message)
        if message.arbitration_id >> 8 == 0xC0:
            response = reply(message.arbitration_id, len(self.offsets), self.direction)
        else:
            index, = struct.unpack(">I", message.data)
            response = reply(message.arbitration_id, index, self.offsets[index])
        self.pending.put(response)

    def recv(self, timeout):
        try:
            return self.pending.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        self.closed = True


class LUTProtocolTests(unittest.TestCase):
    def test_full_10000_sample_capture_is_sequential_big_endian_fd_no_brs(self):
        bus = FirmwareBus(tuple(index / 16 for index in range(10000)))
        progress = []
        capture = LUTReader(bus, driver_id=37).read_all(on_progress=lambda *p: progress.append(p))
        self.assertEqual(capture.metadata, LUTMetadata(10000, -1.0))
        self.assertEqual(capture.offsets, bus.offsets)
        self.assertEqual(capture.driver_id, 37)
        self.assertEqual(bus.sent[0].arbitration_id, 0xC025)
        self.assertEqual(bytes(bus.sent[0].data), b"")
        self.assertEqual(len(bus.sent), 10001)
        for index, message in enumerate(bus.sent[1:]):
            self.assertEqual(message.arbitration_id, 0xC125)
            self.assertEqual(bytes(message.data), struct.pack(">I", index))
        for message in bus.sent:
            self.assertTrue(message.is_extended_id and message.is_fd)
            self.assertFalse(message.bitrate_switch)
        self.assertEqual(progress[-1], (10000, 10000))

    def test_echo_wrong_index_id_and_frame_flags_cannot_become_samples(self):
        class NoisyBus(FirmwareBus):
            def send(self, message, timeout):
                if message.arbitration_id >> 8 == 0xC1:
                    eid = message.arbitration_id
                    index, = struct.unpack(">I", message.data)
                    # Other SocketCAN sockets' eight-byte requests are local TX,
                    # too. A zero-valued genuine LUT reply must still be accepted.
                    self.pending.put(reply(eid, index, 99.0, is_rx=False))
                    self.pending.put(message)  # four-byte RX-labelled echo
                    self.pending.put(reply(eid, index, 99.0, is_fd=False))
                    self.pending.put(reply(eid, index, 99.0, bitrate_switch=True))
                    self.pending.put(reply(eid, index, 99.0, is_extended_id=False))
                    self.pending.put(reply(eid, index, 99.0, dlc=4))
                    self.pending.put(can.Message(arbitration_id=eid, is_extended_id=True,
                                                 is_fd=True, is_remote_frame=True, dlc=8))
                    self.pending.put(reply(eid + 1, index, 99.0))
                    self.pending.put(reply(eid ^ 0x100, index, 99.0))
                    self.pending.put(reply(eid, index + 1, 99.0))
                super().send(message, timeout)

        capture = LUTReader(NoisyBus((0.0, -4.5))).read_all()
        self.assertEqual(capture.offsets, (0.0, -4.5))

    def test_timeout_retries_the_same_index_before_advancing(self):
        class DropFirstBus(FirmwareBus):
            def send(self, message, timeout):
                if message.arbitration_id >> 8 == 0xC1 and len(self.sent) == 1:
                    self.sent.append(message)
                    return
                super().send(message, timeout)

        bus = DropFirstBus()
        retries = []
        capture = LUTReader(bus, timeout=0.003, retries=3,
                            on_retry=lambda *r: retries.append(r)).read_all()
        self.assertEqual(capture.offsets, bus.offsets)
        self.assertEqual([bytes(m.data) for m in bus.sent[1:]],
                         [struct.pack(">I", i) for i in (0, 0, 1)])
        self.assertEqual(retries, [("index 0", 1, 3)])

    def test_exhausted_retries_fail_without_requesting_the_next_index(self):
        class UnresponsiveBus(FirmwareBus):
            def send(self, message, timeout):
                if message.arbitration_id >> 8 == 0xC1:
                    self.sent.append(message)
                else:
                    super().send(message, timeout)

        bus = UnresponsiveBus()
        with self.assertRaisesRegex(TimeoutError, "index 0.*4 attempts"):
            LUTReader(bus, timeout=0.002, retries=3).read_all()
        self.assertEqual(len(bus.sent), 5)  # metadata + initial sample + 3 retries
        self.assertTrue(all(bytes(m.data) == bytes(4) for m in bus.sent[1:]))

    def test_nan_infinity_and_zero_are_preserved_in_capture_and_csv(self):
        values = (math.nan, math.inf, -math.inf, 0.0, -0.0, 2.5)
        capture = LUTReader(FirmwareBus(values)).read_all()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "lut.csv"
            write_lut_csv(path, capture)
            with path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(reader.fieldnames,
                                 ["index", "mechanical_deg", "offset_electrical_deg"])
                rows = list(reader)
            self.assertEqual(len(rows), len(values))
            for index, row in enumerate(rows):
                self.assertEqual(int(row["index"]), index)
                self.assertEqual(float(row["mechanical_deg"]), index * 360.0 / len(values))
                value = float(row["offset_electrical_deg"])
                if math.isnan(values[index]):
                    self.assertTrue(math.isnan(value))
                else:
                    self.assertEqual(value, values[index])
            self.assertEqual(math.copysign(1, float(rows[4]["offset_electrical_deg"])), -1)
            original = path.read_bytes()
            with self.assertRaises(FileExistsError):
                write_lut_csv(path, capture)
            self.assertEqual(path.read_bytes(), original)

    def test_current_csv_angle_mapping_and_incomplete_capture_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "lut.csv"
            capture = LUTCapture(0, LUTMetadata(10000, 1.0), (0.0,) * 10000)
            write_lut_csv(path, capture)
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            for index, angle in ((0, 0.0), (2500, 90.0), (5000, 180.0),
                                 (7500, 270.0), (9999, 359.964)):
                self.assertAlmostEqual(float(rows[index]["mechanical_deg"]), angle)
            incomplete_path = Path(folder) / "partial.csv"
            with self.assertRaisesRegex(ValueError, "incomplete"):
                write_lut_csv(incomplete_path, LUTCapture(0, capture.metadata, (0.0,)))
            self.assertFalse(incomplete_path.exists())

    def test_cancellation_stops_before_the_next_request(self):
        stop = threading.Event()
        bus = FirmwareBus()
        with self.assertRaises(LUTCancelled):
            LUTReader(bus, stop_event=stop).read_all(on_progress=lambda *_: stop.set())
        self.assertEqual(len(bus.sent), 2)  # only metadata and sample 0

    def test_transport_and_bus_off_errors_fail_without_retries(self):
        bus = FirmwareBus()
        with patch.object(bus, "send", side_effect=can.CanOperationError("TX failed")):
            with self.assertRaisesRegex(can.CanOperationError, "TX failed"):
                LUTReader(bus).read_all()

        class BusOffBus(FirmwareBus):
            def send(self, message, timeout):
                self.sent.append(message)
                self.pending.put(can.Message(arbitration_id=0x40, is_error_frame=True))

        bus = BusOffBus()
        with self.assertRaisesRegex(RuntimeError, "BUS-OFF"):
            LUTReader(bus).read_all()
        self.assertEqual(len(bus.sent), 1)

    def test_invalid_metadata_does_not_trigger_sample_requests(self):
        for count in (0, 10001, 0xFFFFFFFF):
            bus = FirmwareBus()
            with patch.object(bus, "send", side_effect=lambda message, timeout:
                              bus.pending.put(reply(message.arbitration_id, count, 1.0))):
                with self.assertRaisesRegex(ValueError, "sample count"):
                    LUTReader(bus).read_all()


class LUTGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def pump_until(self, predicate, timeout=2):
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
        with patch("LUT.QtCore.QSettings", return_value=settings):
            window = LUTWindow()
        window.interface.setCurrentText("slcan")
        window.channel.setEditText("COM_TEST")
        window.driver_id.setValue(0)
        return window

    def test_collection_runs_in_worker_and_enables_csv_only_after_shutdown(self):
        window = self.window()
        bus = FirmwareBus((0.0, math.nan, math.inf, -math.inf, 2.5))
        try:
            with patch("LUT.can.Bus", return_value=bus) as factory:
                window._start_read()
                self.assertFalse(window.save.isEnabled())
                self.pump_until(lambda: window.worker is None)
            self.assertTrue(bus.closed)
            self.assertTrue(window.save.isEnabled())
            self.assertEqual(len(window.capture.offsets), 5)
            self.assertIn("NaN 1", window.stats.text())
            self.assertIn("+Inf 1", window.stats.text())
            self.assertIn("-Inf 1", window.stats.text())
            self.assertIn("수집 완료", window.status.text())
            self.assertEqual(factory.call_args.kwargs["interface"], "slcan")
        finally:
            window.close()
            window.deleteLater()

    def test_gui_remains_responsive_and_close_waits_for_cancel_cleanup(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingBus(FirmwareBus):
            def send(self, message, timeout):
                entered.set()
                release.wait(2)
                super().send(message, timeout)

        window = self.window()
        bus = BlockingBus()
        beats = []
        heartbeat = QtCore.QTimer()
        heartbeat.timeout.connect(lambda: beats.append(time.monotonic()))
        heartbeat.start(5)
        try:
            with patch("LUT.can.Bus", return_value=bus):
                window._start_read()
                self.pump_until(entered.is_set)
                self.pump_until(lambda: len(beats) >= 5)
                self.assertFalse(window.close())
                self.assertIsNotNone(window.worker)
                release.set()
                self.pump_until(lambda: window.worker is None)
            self.assertTrue(bus.closed)
            self.assertIsNone(window.capture)
            self.assertFalse(window.save.isEnabled())
        finally:
            release.set()
            heartbeat.stop()
            if window.worker is not None:
                window._cancel_read()
                self.pump_until(lambda: window.worker is None)
            window.close()
            window.deleteLater()

    def test_failed_capture_and_metadata_only_do_not_enable_csv(self):
        window = self.window()
        bus = FirmwareBus()
        try:
            with patch("LUT.can.Bus", return_value=bus):
                window._start_read(metadata_only=True)
                self.pump_until(lambda: window.worker is None)
            self.assertTrue(bus.closed)
            self.assertEqual(len(bus.sent), 1)
            self.assertIsNone(window.capture)
            self.assertFalse(window.save.isEnabled())
            with patch("LUT.can.Bus", side_effect=RuntimeError("adapter unavailable")):
                window._start_read()
                self.pump_until(lambda: window.worker is None)
            self.assertIn("adapter unavailable", window.status.text())
            self.assertFalse(window.save.isEnabled())
        finally:
            window.close()
            window.deleteLater()

    def test_all_zero_and_all_nonfinite_luts_are_complete_diagnostic_captures(self):
        for values, diagnostic in (((0.0, 0.0), "전체 값이 0"),
                                   ((math.nan, math.inf, -math.inf), "유한값 0/3")):
            with self.subTest(values=values):
                window = self.window()
                bus = FirmwareBus(values)
                try:
                    with patch("LUT.can.Bus", return_value=bus):
                        window._start_read()
                        self.pump_until(lambda: window.worker is None)
                    self.assertIn(diagnostic, window.stats.text())
                    self.assertTrue(window.save.isEnabled())
                    self.assertEqual(len(window.capture.offsets), len(values))
                finally:
                    window.close()
                    window.deleteLater()


if __name__ == "__main__":
    unittest.main()
