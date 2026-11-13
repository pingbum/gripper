"""Position + feedforward current payloads and feed_forward.py GUI gating; no hardware."""
import math
import os
from pathlib import Path
import struct
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from PyQt5 import QtWidgets
from config import MODE_CURRENT, MODE_POSITION
from feed_forward import FF_SECTION, FeedForwardApp
from position_control_app import FINGER_SECTIONS
from protocol import pack_command_payload, pack_position_payload


class PositionPayloadTests(unittest.TestCase):
    def test_layout_units_and_sign(self):
        self.assertEqual(pack_position_payload(30.0, 0.2).hex(" "), "00 00 0b b8 00 c8 00 00")
        self.assertEqual(pack_position_payload(30.0, -0.5)[4:6].hex(), "fe0c")
        self.assertEqual(pack_position_payload(-12.34, 0.001), struct.pack(">ihH", -1234, 1, 0))
        self.assertEqual(pack_position_payload(0, 32.767)[4:6].hex(), "7fff")
        self.assertEqual(pack_position_payload(0, -32.768)[4:6].hex(), "8000")

    def test_zero_feedforward_matches_legacy_position_payload(self):
        for deg in (0.0, 12.345, -720.0, 3000.0):
            self.assertEqual(pack_position_payload(deg), pack_command_payload(MODE_POSITION, deg))

    def test_invalid_values_raise(self):
        for args in ((math.nan, 0), (0, math.nan), (math.inf, 0), (0, -math.inf), (0, 32.768), (0, -32.769)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                pack_position_payload(*args)


class FeedForwardGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.window = w = FeedForwardApp()
        w.timer.stop()
        w.ramp_timer.stop()
        self.sent = []
        w._open_bus = lambda: None
        w._close_bus = lambda: None
        w._send = lambda mode, motor_id, payload: self.sent.append((mode, motor_id, bytes(payload))) or True
        self.sec = w.sections[FF_SECTION]
        now = time.monotonic()
        for i in (0, 1, 2, 3):
            w.status[i] = (now, 0.0, 0, 0.0)
        self.sec.cb_connect.setChecked(True)
        w.spin_ff[1].setValue(0.2)
        w.spin_ff[2].setValue(-0.15)

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()

    def positions(self, motor_id=None):
        """Sent position frames as (motor_id, centideg, i_ff mA, reserved)."""
        frames = []
        for mode, i, payload in self.sent:
            if mode == MODE_POSITION and (motor_id is None or i == motor_id):
                self.assertEqual(len(payload), 8)
                frames.append((i, *struct.unpack(">ihH", payload)))
        return frames

    def last_ff(self):
        return {i: self.positions(i)[-1][2] for i in (1, 2)}

    def finish_softstart(self):
        self.window.softstart[self.sec] = time.monotonic() - 100.0
        self.window._ramp()

    def test_feedforward_waits_for_softstart_then_steps_in(self):
        w = self.window
        w.btn_ff.setChecked(True)
        w.spin_softstart.setValue(0.3)
        self.sec.cb_control.setChecked(True)
        self.assertIn(self.sec, w.softstart)
        w._ramp()  # still soft-starting
        self.assertTrue(self.positions())
        self.assertTrue(all(ma == 0 for _, _, ma, _ in self.positions()))
        self.sent.clear()
        self.finish_softstart()
        self.assertNotIn(self.sec, w.softstart)
        self.assertEqual(self.last_ff(), {1: 200, 2: -150})
        self.assertEqual(w.ff_sent, {1: 0.2, 2: -0.15})
        # Applied once; later ticks with no change send nothing new.
        self.sent.clear()
        w._ramp()
        self.assertEqual(self.positions(), [])

    def test_zero_softstart_applies_after_first_zero_command(self):
        w = self.window
        w.btn_ff.setChecked(True)
        w.spin_softstart.setValue(0.0)
        self.sec.cb_control.setChecked(True)
        frames = self.positions(1)
        self.assertEqual([ma for _, _, ma, _ in frames], [0, 200])

    def test_value_change_and_toggle_resend_last_position_immediately(self):
        w = self.window
        w.btn_smooth.setChecked(True)
        w.spin_softstart.setValue(0.0)
        self.sec.cb_control.setChecked(True)
        self.assertEqual(self.last_ff(), {1: 0, 2: 0})
        cmd = self.positions(1)[-1][1]
        w.btn_ff.setChecked(True)
        self.assertEqual(self.last_ff(), {1: 200, 2: -150})
        w.spin_ff[2].setValue(-0.3)
        self.assertEqual(self.last_ff(), {1: 200, 2: -300})
        self.assertEqual(self.positions(1)[-1][1], cmd)  # same position, new current only
        w.btn_ff.setChecked(False)
        self.assertEqual(self.last_ff(), {1: 0, 2: 0})
        self.assertEqual(w.spin_ff[2].value(), -0.3)  # input kept for the next ON

    def test_single_selected_motor_gets_zero(self):
        w = self.window
        w.btn_ff.setChecked(True)
        w.spin_softstart.setValue(0.0)
        self.sec.cb_motors[2].setChecked(False)
        self.sec.cb_control.setChecked(True)
        self.finish_softstart()
        self.assertTrue(self.positions(1))
        self.assertEqual(self.positions(2), [])
        self.assertTrue(all(ma == 0 for _, _, ma, _ in self.positions()))
        self.assertIn("M1, M2", w.lbl_ff.text())

    def test_control_off_releases_with_zero_current_and_clears_state(self):
        w = self.window
        w.btn_ff.setChecked(True)
        w.spin_softstart.setValue(0.0)
        self.sec.cb_control.setChecked(True)
        self.sent.clear()
        self.sec.cb_control.setChecked(False)
        self.assertEqual(self.positions(), [])
        self.assertEqual({(m, i, p) for m, i, p in self.sent},
                         {(MODE_CURRENT, i, pack_command_payload(MODE_CURRENT, 0.0)) for i in (1, 2)})
        self.assertEqual(w.ff_sent, {1: 0.0, 2: 0.0})
        w._ramp()
        self.assertEqual(self.positions(), [])

    def test_other_joints_never_carry_feedforward(self):
        w = self.window
        w.btn_ff.setChecked(True)
        w.spin_softstart.setValue(0.0)
        self.sec.cb_control.setChecked(True)
        dip = w.sections[FINGER_SECTIONS[1]]
        dip.cb_connect.setChecked(True)
        dip.cb_control.setChecked(True)
        w._command(dip, 10.0)
        self.assertTrue(self.positions(3))
        self.assertTrue(all(ma == 0 for _, _, ma, _ in self.positions(3)))

    def test_smoothing_ramp_and_initialize_keep_feedforward(self):
        w = self.window
        w.btn_ff.setChecked(True)
        w.btn_smooth.setChecked(True)
        w.spin_softstart.setValue(0.0)
        self.sec.cb_control.setChecked(True)
        self.sec.sld_ref.setValue(self.sec.sld_ref.value() + 100)  # +10 deg target
        self.sent.clear()
        w._ramp()
        self.assertTrue(self.positions(1))
        self.assertEqual(self.last_ff(), {1: 200, 2: -150})
        self.sent.clear()
        w._on_initialize()
        self.assertEqual(self.positions(1)[-1][1:3], (0, 200))


if __name__ == "__main__":
    unittest.main()
