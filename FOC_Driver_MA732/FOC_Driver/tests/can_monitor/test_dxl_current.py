"""Mode 5 register sequencing, fault handling and inherited Qt UI; no hardware."""
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from PyQt5 import QtWidgets
from dxl_current import (CurrentPositionWorker, DxlCurrentApp, DxlCurrentSettings,
                         DxlError, U2D2CurrentDynamixel, dxl_signed16)


class FakeDxl:
    def __init__(self, mode=4, drive=13, current_limit=910, position_after_mode=None):
        self.reg = {0: 1710, 10: drive, 11: mode, 38: current_limit, 64: 0,
                    80: 0, 82: 10, 84: 2000, 102: current_limit, 108: 0, 112: 0,
                    116: 0, 126: (-25) & 0xFFFF, 132: 100}
        self.calls = []
        self.position_after_mode = position_after_mode
        self.fail_read = None
        self.bad_readback = None
        self.on_goal_read = None
        self.closed = False

    def ping(self, dxl_id):
        return self.reg[0]

    def close(self):
        self.closed = True

    def torque(self, dxl_id, on):
        self.calls.append(('torque', dxl_id, on))
        self.reg[64] = int(on)

    def _read(self, width, dxl_id, address):
        self.calls.append((f'r{width}', dxl_id, address))
        if self.fail_read == address:
            raise DxlError('simulated read failure')
        if address == 116 and self.on_goal_read:
            self.on_goal_read()
        value = self.reg[address]
        return value + 1 if self.bad_readback == address else value

    def r1(self, dxl_id, address):
        assert address in (10, 11, 64)
        return self._read(1, dxl_id, address)

    def r2(self, dxl_id, address):
        assert address in (0, 38, 80, 82, 84, 102, 126)
        return self._read(2, dxl_id, address)

    def r4(self, dxl_id, address):
        assert address in (108, 112, 116, 132)
        return self._read(4, dxl_id, address)

    def w1(self, dxl_id, address, value):
        assert address in (10, 11)
        assert self.reg[64] == 0, 'EEPROM write while torque is ON'
        self.calls.append(('w1', dxl_id, address, value))
        self.reg[address] = value
        if address == 11:
            self.reg.update({80: 0, 82: 0, 84: 2000, 102: self.reg[38], 108: 0, 112: 0})
            if self.position_after_mode is not None:
                self.reg[132] = self.position_after_mode

    def w2(self, dxl_id, address, value):
        assert address in (80, 82, 84, 102), 'do not overwrite EEPROM Current Limit'
        self.calls.append(('w2', dxl_id, address, value))
        self.reg[address] = value

    def w4(self, dxl_id, address, value):
        assert address in (108, 112, 116)
        self.calls.append(('w4', dxl_id, address, value))
        self.reg[address] = value & 0xFFFFFFFF


class Mode5WorkerTests(unittest.TestCase):
    def test_register_widths_sign_and_input_validation(self):
        dxl = U2D2CurrentDynamixel.__new__(U2D2CurrentDynamixel)
        dxl.packet, dxl._call = Mock(), Mock()
        dxl.w2(1, 102, 100)
        dxl._call.assert_called_once_with(dxl.packet.write2ByteTxRx, 'write2 ID 1@102', 1, 102, 100)
        for value in (-1, 65536, 1.5, True):
            with self.assertRaises(ValueError):
                dxl.w2(1, 102, value)
        for raw, expected in ((0, 0), (32767, 32767), (32768, -32768), (65535, -1)):
            self.assertEqual(dxl_signed16(raw), expected)
        for settings in (DxlCurrentSettings(-1), DxlCurrentSettings(911),
                         DxlCurrentSettings(p=16384), DxlCurrentSettings(i=-1),
                         DxlCurrentSettings(d=1.5)):
            with self.assertRaises(ValueError):
                settings.validate()

    def test_prepare_keeps_torque_off_and_seeds_after_mode_position_on_activation(self):
        dxl = FakeDxl(position_after_mode=120)
        worker = CurrentPositionWorker(dxl, 1)
        worker._prepare()
        self.assertEqual(dxl.reg[10], 1)  # Preserve reverse direction; clear profile/auto-ON bits.
        self.assertEqual(dxl.reg[11], 5)
        self.assertEqual([dxl.reg[a] for a in (84, 82, 80, 102, 112, 108)], [800, 0, 500, 100, 45, 15])
        self.assertFalse(dxl.reg[64])
        self.assertTrue(worker.snapshot()['ready'])
        self.assertEqual(worker.sample[1], 120)
        self.assertEqual(worker.current_sample[1], -25)
        self.assertFalse(any(call[:3] == ('w4', 1, 116) for call in dxl.calls))
        mode_write = dxl.calls.index(('w1', 1, 11, 5))
        self.assertTrue(all(index > mode_write for index, call in enumerate(dxl.calls) if call[0] == 'w2'))
        dxl.reg[132] = 130  # Move by hand after connection, before enabling position control.
        worker.begin((0, 1024), DxlCurrentSettings())
        worker._step()
        self.assertEqual(dxl.reg[116], 130)
        self.assertEqual(worker.snapshot()['activation'][1], 130)
        self.assertTrue(dxl.reg[64])
        dxl.calls.clear()
        worker.request(True, 250, (0, 1024))
        worker._step()
        self.assertEqual(dxl.calls, [('w4', 1, 116, 250)])

    def test_ready_mode_does_not_rewrite_eeprom_and_reenable_seeds_current_position(self):
        dxl = FakeDxl(mode=5, drive=1)
        worker = CurrentPositionWorker(dxl, 1)
        worker._prepare()
        self.assertFalse(any(call[0] == 'w1' for call in dxl.calls))
        worker.begin((0, 1024), DxlCurrentSettings())
        worker._step()
        worker.request(False)
        worker._step()
        self.assertFalse(dxl.reg[64])
        dxl.reg[132] = 222
        worker.begin((0, 1024), DxlCurrentSettings())
        worker._step()
        self.assertEqual(dxl.reg[116], 222)
        self.assertFalse(any(call[0] == 'w1' for call in dxl.calls))

    def test_settings_readback_and_actual_current_ceiling(self):
        dxl = FakeDxl(current_limit=150)
        worker = CurrentPositionWorker(dxl, 1)
        worker._prepare()
        worker.begin((0, 1024), DxlCurrentSettings())
        worker._step()
        with self.assertRaises(ValueError):
            worker.request_settings(DxlCurrentSettings(current_ma=151))
        for current in (50, 140):
            dxl.calls.clear()
            worker.request_settings(DxlCurrentSettings(current, p=700, d=400))
            worker._step()
            writes = [call for call in dxl.calls if call[0] == 'w2']
            self.assertEqual((writes[0] if current == 50 else writes[-1])[2], 102)
            self.assertEqual(dxl.reg[102], current)
            self.assertEqual(worker.snapshot()['applied'].current_ma, current)
            self.assertTrue(dxl.reg[64])

    def test_rapid_off_on_still_reseeds_instead_of_leaving_gui_waiting(self):
        dxl = FakeDxl()
        worker = CurrentPositionWorker(dxl, 1)
        worker._prepare()
        worker.begin((0, 1024), DxlCurrentSettings())
        worker._step()
        previous = worker.snapshot()['activation']
        worker.request(False)
        dxl.reg[132] = 220
        worker.begin((0, 1024), DxlCurrentSettings())
        dxl.calls.clear()
        worker._step()
        self.assertEqual(dxl.calls[0], ('torque', 1, False))
        self.assertTrue(dxl.reg[64])
        self.assertEqual(dxl.reg[116], 220)
        self.assertNotEqual(worker.snapshot()['activation'], previous)

    def test_negative_extended_position_is_encoded_and_decoded(self):
        dxl = FakeDxl(position_after_mode=(-123) & 0xFFFFFFFF)
        worker = CurrentPositionWorker(dxl, 1)
        worker._prepare()
        self.assertEqual(worker.sample[1], -123)
        worker.begin((-200, 200), DxlCurrentSettings())
        worker._step()
        self.assertEqual(dxl.reg[116], (-123) & 0xFFFFFFFF)
        self.assertEqual(worker.snapshot()['activation'][1], -123)

    def test_read_failure_after_torque_on_turns_torque_off_and_stops_thread(self):
        dxl = FakeDxl()
        worker = CurrentPositionWorker(dxl, 1)
        worker.start()
        try:
            deadline = time.monotonic() + 2
            while not worker.snapshot()['ready'] and time.monotonic() < deadline:
                time.sleep(0.002)
            self.assertTrue(worker.snapshot()['ready'])

            def fail_next_current_read():
                dxl.fail_read = 126

            dxl.on_goal_read = fail_next_current_read
            worker.begin((0, 1024), DxlCurrentSettings())
            worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertIn(('torque', 1, True), dxl.calls)
            self.assertFalse(dxl.reg[64])
            self.assertTrue(worker.snapshot()['fault'])
            self.assertFalse(worker.snapshot()['ready'])
        finally:
            worker.stop_event.set()
            worker.join(2)

    def test_cancel_and_stop_during_activation_never_enable_torque(self):
        for cancel in ('off', 'restart', 'stop'):
            with self.subTest(cancel=cancel):
                dxl = FakeDxl()
                worker = CurrentPositionWorker(dxl, 1)
                worker._prepare()
                worker.begin((0, 1024), DxlCurrentSettings())
                actions = {'off': lambda: worker.request(False),
                           'restart': lambda: worker.begin((0, 1024), DxlCurrentSettings()),
                           'stop': worker.stop_event.set}
                dxl.on_goal_read = actions[cancel]
                worker._step()
                self.assertNotIn(('torque', 1, True), dxl.calls)

    def test_faults_latch_and_exit_without_automatic_retry(self):
        for failure in ('model', 'limit', 'readback', 'read'):
            with self.subTest(failure=failure):
                dxl = FakeDxl()
                if failure == 'model':
                    dxl.reg[0] = 999
                elif failure == 'limit':
                    dxl.reg[38] = 50
                elif failure == 'readback':
                    dxl.bad_readback = 102
                else:
                    dxl.fail_read = 126
                worker = CurrentPositionWorker(dxl, 1)
                worker.run()
                self.assertTrue(worker.snapshot()['fault'])
                self.assertFalse(worker.snapshot()['ready'])
                self.assertFalse(dxl.reg[64])
                self.assertNotIn(('torque', 1, True), dxl.calls)
                self.assertIsNone(worker.sample)
                with self.assertRaises(DxlError):
                    worker.begin((0, 1024), DxlCurrentSettings())

    def test_bounds_and_changed_mode_block_activation(self):
        for changed_mode in (False, True):
            with self.subTest(changed_mode=changed_mode):
                dxl = FakeDxl()
                worker = CurrentPositionWorker(dxl, 1)
                worker._prepare()
                worker.begin((0, 50) if not changed_mode else (0, 1024), DxlCurrentSettings())
                if changed_mode:
                    dxl.reg[11] = 4
                with self.assertRaises(DxlError):
                    worker._step()
                self.assertNotIn(('torque', 1, True), dxl.calls)


class Mode5GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.window = DxlCurrentApp()
        self.window.timer.stop()
        self.window.ramp_timer.stop()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()

    def pump_until(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.window._poll()
            if predicate():
                return
            time.sleep(0.002)
        self.fail('GUI/worker operation timed out')

    def connect_fake(self, dxl):
        with patch('dxl_current.U2D2CurrentDynamixel', return_value=dxl):
            self.window.mcp.cb_connect.setChecked(True)

    def test_gui_connection_activation_settings_initialize_and_disconnect(self):
        window, dxl = self.window, FakeDxl(current_limit=200, position_after_mode=120)
        self.assertTrue(window.btn_kd_guard.isChecked())
        self.assertEqual(len(window.sections), 3)
        self.assertFalse(window.btn_apply_dxl.isEnabled())
        self.connect_fake(dxl)
        self.pump_until(lambda: window.btn_apply_dxl.isEnabled())
        self.assertFalse(dxl.reg[64])
        self.assertEqual(window.spin_current.maximum(), 200)
        self.assertIn('-25 mA', window.lbl_current.text())
        window.mcp.cb_control.setChecked(True)
        self.assertIsNone(window.cmd.get(window.mcp))
        self.assertFalse(window.mcp.sld_ref.isEnabled())
        self.assertFalse(window._command(window.mcp, 90))
        self.pump_until(lambda: window.mcp.sld_ref.isEnabled())
        self.assertEqual(dxl.reg[116], 120)
        self.assertTrue(dxl.reg[64])
        window.mcp.sld_ref.setValue(200)  # 20 degrees
        expected = window._deg_to_raw(20)
        self.pump_until(lambda: dxl.reg[116] == expected)
        window.spin_current.setValue(150)
        window.spin_p.setValue(700)
        window.btn_apply_dxl.click()
        self.pump_until(lambda: '150 mA' in window.lbl_dxl_mode.text())
        self.assertEqual((dxl.reg[102], dxl.reg[84]), (150, 700))
        # Initialize changes the software origin, not the physical goal at the current position.
        self.pump_until(lambda: window._mcp_sample() is not None)
        window._on_initialize()
        self.pump_until(lambda: dxl.reg[116] == dxl.reg[132])
        self.assertEqual(window.mcp.ref_deg, 0)
        window.mcp.cb_connect.setChecked(False)
        self.assertFalse(dxl.reg[64])
        self.assertTrue(dxl.closed)
        self.assertIsNone(window.dxl_worker)
        self.assertFalse(window.btn_apply_dxl.isEnabled())

    def test_fault_disables_control_and_requires_reconnect(self):
        window, dxl = self.window, FakeDxl()
        dxl.bad_readback = 102
        self.connect_fake(dxl)
        self.pump_until(lambda: '오류' in window.lbl_dxl_mode.text())
        self.assertFalse(window.mcp.cb_control.isEnabled())
        self.assertFalse(window.mcp.cb_control.isChecked())
        self.assertFalse(window.btn_apply_dxl.isEnabled())
        self.assertFalse(dxl.reg[64])
        worker = window.dxl_worker
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertNotIn(('torque', 1, True), dxl.calls)


if __name__ == '__main__':
    unittest.main()
