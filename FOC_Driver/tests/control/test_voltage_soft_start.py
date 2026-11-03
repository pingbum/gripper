"""Test the voltage soft start in the built ARM ELF, without motor hardware.

Build Debug, then run: python tests/control/test_voltage_soft_start.py
Uses the Unicorn/pyelftools harness from test_position_gains.py. ADC/timer
registers are simulated; electrical timing and motor stability are not modeled.
"""
import struct
import unittest

from test_position_gains import Firmware


class CurrentFirmware(Firmware):
    def __init__(self):
        super().__init__()
        self.cpu.mem_map(0x40000000, 0x20000)  # TIM2, HRTIM1.
        self.cpu.mem_map(0x50000000, 0x1000)   # ADC1/2/3.
        self.cpu.mem_write(self.symbols["htim2"], struct.pack("<I", 0x40000000))
        self.position = self.symbols["state"] + 16
        # position_instance_f32_t: seven 32-bit fields, 10000 offsets, direction, p.
        self.cpu.mem_write(self.position + 28 + 10000 * 4, struct.pack("<ff", 1.0, 7.0))
        self.set_adc(4095, 0, 4095)  # Nonzero d/q current errors, even at iq_ref=0.
        self.call("_Z17current_loop_initv")
        self.full_limit = self.controller("id_control")[5]

    def set_adc(self, a, b, c):
        for base, count in zip((0x50000000, 0x50000400, 0x50000100), (a, b, c)):
            self.cpu.mem_write(base + 0x80, struct.pack("<I", count))

    def floats(self, symbol, count):
        return struct.unpack(f"<{count}f", self.cpu.mem_read(self.symbols[symbol], count * 4))

    def controller(self, symbol):
        # Previous error, integral, derivative state[2], lower/upper bounds, Kp/Ki/Kd.
        return self.floats(symbol, 9)

    def step(self, command=0.0):
        self.call("_Z12current_loopP23position_instance_f32_tf", self.position, floating=command)
        return self.floats("v_command", 2)


class VoltageSoftStartTests(unittest.TestCase):
    def test_total_voltage_and_integrators_ramp_then_restore_without_reset(self):
        firmware = CurrentFirmware()
        for cycle in range(10000):
            output = firmware.step(0.8)
            limit = firmware.full_limit * cycle / 10000
            for voltage in output:
                self.assertLessEqual(abs(voltage), limit + 2e-6)
            for controller in ("id_control", "iq_control"):
                state = firmware.controller(controller)
                self.assertLessEqual(abs(state[1]), limit + 2e-6)
                self.assertAlmostEqual(state[4], -limit, places=5)
                self.assertAlmostEqual(state[5], limit, places=5)
            if cycle == 0:
                self.assertEqual(output, (0.0, 0.0))
                self.assertEqual(firmware.floats("duty", 3), (0.5, 0.5, 0.5))
            elif cycle == 5000:  # 100 ms at 50 kHz.
                self.assertAlmostEqual(output[0], -firmware.full_limit / 2, places=5)
                self.assertAlmostEqual(output[1], firmware.full_limit / 2, places=5)

        # At 200 ms restore the original bounds, retaining accumulated state.
        output = firmware.step(0.8)
        for controller in ("id_control", "iq_control"):
            state = firmware.controller(controller)
            self.assertEqual(state[4:6], (-firmware.full_limit, firmware.full_limit))
            self.assertGreater(abs(state[1]), firmware.full_limit * 0.99)
        # As requested, normal operation retains the original unclamped P+I behavior.
        self.assertGreater(abs(output[1]), firmware.full_limit)

    def test_cross_coupling_is_included_in_both_axis_limits(self):
        for speed in (-2e7, 2e7):
            with self.subTest(electrical_speed=speed):
                firmware = CurrentFirmware()
                # Keep extrapolated electrical angle at zero with a stationary
                # simulated timer while forcing a large cross-coupling term.
                firmware.cpu.mem_write(firmware.position + 20,
                                       struct.pack("<ff", -speed * 0.0001, speed))
                for cycle in range(101):
                    output = firmware.step()
                    limit = firmware.full_limit * cycle / 10000
                    for voltage in output:
                        self.assertLessEqual(abs(voltage), limit + 2e-6)
                for voltage in firmware.floats("v_cross", 2):
                    self.assertGreater(abs(voltage), firmware.full_limit * 2)

    def test_reinitializing_restarts_at_zero_and_clears_integrators(self):
        firmware = CurrentFirmware()
        for _ in range(500):
            firmware.step(0.8)
        self.assertGreater(abs(firmware.controller("iq_control")[1]), 0.0)
        firmware.call("_Z17current_loop_initv")
        self.assertEqual(firmware.step(0.8), (0.0, 0.0))
        for controller in ("id_control", "iq_control"):
            self.assertEqual(firmware.controller(controller)[1], 0.0)

    def test_can_bandwidth_update_does_not_restart_or_bypass_ramp(self):
        firmware = CurrentFirmware()
        for _ in range(5000):
            firmware.step(0.8)
        previous_kp = firmware.controller("iq_control")[6]
        firmware.command(0x11, struct.pack(">f", 1000.0))
        self.assertNotEqual(firmware.controller("iq_control")[6], previous_kp)
        output = firmware.step(0.8)
        self.assertAlmostEqual(output[1], firmware.full_limit / 2, places=5)
        self.assertAlmostEqual(firmware.controller("iq_control")[5],
                               firmware.full_limit / 2, places=5)


if __name__ == "__main__":
    unittest.main()
