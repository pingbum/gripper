"""Exercise CAN position commands through the built ARM firmware and TIM7.

Build Debug, then run: python -m unittest discover -s tests/control -v
No MCU or CAN device is opened. Physical timing and motor response are not modeled.
"""
import struct
import unittest

from elftools.elf.elffile import ELFFile
from unicorn import UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_PRIMASK, UC_ARM_REG_S0

from test_position_gains import ELF_PATH, Firmware


class PositionFirmware(Firmware):
    def __init__(self):
        super().__init__()
        # Read member offsets from debug info instead of duplicating State_t.
        self.fields = {}
        with ELF_PATH.open("rb") as stream:
            dwarf = ELFFile(stream).get_dwarf_info()
            for unit in dwarf.iter_CUs():
                for die in unit.iter_DIEs():
                    name = die.attributes.get("DW_AT_name")
                    if die.tag != "DW_TAG_structure_type" or not name or name.value != b"State_t":
                        continue
                    for member in die.iter_children():
                        offset = member.attributes.get("DW_AT_data_member_location")
                        if member.tag == "DW_TAG_member" and offset:
                            field = member.attributes["DW_AT_name"].value.decode()
                            self.fields[field] = self.symbols["state"] + offset.value
                    if "i_ff" in self.fields:
                        break
                if "i_ff" in self.fields:
                    break
        if "i_ff" not in self.fields:
            raise RuntimeError("Build the new firmware with debug symbols first")
        self.tick_symbol = next(s for s in self.symbols if "TIM7_PeriodElapsedCB" in s)
        control = self.symbols["_ZL16position_control"]
        self.limit = struct.unpack("<f", self.cpu.mem_read(control + 20, 4))[0]
        self.call("_Z18velocity_loop_initv")
        # position_instance_f32_t: seven fields, encoder LUT, direction, pole pairs.
        self.cpu.mem_write(self.fields["position"] + 28 + 10000 * 4,
                           struct.pack("<ff", 1, 7))

    def value(self, field):
        return struct.unpack("<f", self.cpu.mem_read(self.fields[field], 4))[0]

    def mode(self):
        return struct.unpack("<I", self.cpu.mem_read(self.fields["CONTROL_MODE"], 4))[0]

    def set_gains(self, kp=0, kd=0):
        self.command(0x22, struct.pack(">f", kp))
        self.command(0x23, struct.pack(">f", kd))

    def position_command(self, degrees, milliamps, reserved=0):
        self.command(0x05, struct.pack(">ihH", round(degrees * 100), milliamps, reserved))

    def tick(self):
        self.call(self.tick_symbol, 0)
        return self.value("iq_ref")


class PositionFeedforwardTests(unittest.TestCase):
    def setUp(self):
        self.firmware = PositionFirmware()
        self.firmware.set_gains()

    def test_signed_big_endian_units_and_reserved_bytes(self):
        firmware = self.firmware
        self.assertEqual(firmware.value("i_ff"), 0)
        for milliamps in (0, 1, -1, 50, -50, 500, -500, 32767, -32768):
            with self.subTest(milliamps=milliamps):
                firmware.position_command(-12.34, milliamps, reserved=0xABCD)
                self.assertAlmostEqual(firmware.value("theta_ref"), -12.34, places=5)
                self.assertAlmostEqual(firmware.value("i_ff"), milliamps * 0.001, places=5)
                self.assertEqual(firmware.mode(), 0x05)

    def test_reject_incomplete_position_packets_and_wrong_driver(self):
        firmware = self.firmware
        firmware.position_command(20, -500)
        expected = (firmware.value("theta_ref"), firmware.value("i_ff"), firmware.mode())
        for length in (0, 1, 2, 4, 6, 12, 64):
            firmware.command(0x05, bytes(length))
            self.assertEqual((firmware.value("theta_ref"), firmware.value("i_ff"), firmware.mode()), expected)
        firmware.command(0x05, struct.pack(">ihH", 100, 100, 0), driver_id=1)
        self.assertEqual((firmware.value("theta_ref"), firmware.value("i_ff"), firmware.mode()), expected)

    def test_feedforward_is_held_without_accumulation_and_zero_clears_it(self):
        firmware = self.firmware
        for milliamps in (1, -1, 500, -500, 0):
            firmware.position_command(0, milliamps)
            for _ in range(10):
                self.assertAlmostEqual(firmware.tick(), milliamps * 0.001, places=6)

    def test_zero_padding_retains_position_control(self):
        firmware = self.firmware
        firmware.set_gains(kp=0.1)
        firmware.command(0x05, struct.pack(">i", 300) + bytes(4))
        self.assertAlmostEqual(firmware.tick(), 0.3, places=6)

    def test_pd_and_feedforward_cancel_before_any_current_limit(self):
        firmware = self.firmware
        firmware.set_gains(kp=0.1)
        for degrees, milliamps, expected in ((20, -1000, 1), (-20, 1000, -1),
                                            (-20, 2500, 0.5), (20, -2500, -0.5)):
            firmware.position_command(degrees, milliamps)
            self.assertAlmostEqual(firmware.tick(), expected, places=6)

    def test_only_the_final_clamped_reference_is_published(self):
        firmware = self.firmware
        firmware.set_gains(kp=0.1)
        published = []

        def on_publish(cpu, access, address, size, value, context):
            self.assertEqual(size, 4)
            published.append(struct.unpack("<f", struct.pack("<I", value))[0])

        address = firmware.fields["iq_ref"]
        firmware.cpu.hook_add(UC_HOOK_MEM_WRITE, on_publish, begin=address, end=address + 3)
        for degrees, milliamps, sign in ((200, 32767, 1), (-200, -32768, -1),
                                        (200, 0, 1), (-200, 0, -1)):
            firmware.position_command(degrees, milliamps)
            self.assertAlmostEqual(firmware.tick(), sign * firmware.limit, places=6)
        self.assertEqual(len(published), 4)
        self.assertTrue(all(abs(value) <= firmware.limit for value in published))

    def test_cogging_is_included_before_the_final_limit(self):
        firmware = self.firmware
        firmware.set_gains(kp=0.1)
        cogging = -0.5

        def cogging_current(cpu, address, size, context):
            cpu.reg_write(UC_ARM_REG_S0, struct.unpack("<I", struct.pack("<f", cogging))[0])
            firmware.return_from_hal(0)

        firmware.hook("_Z28cogging_compensation_currentf", cogging_current)
        firmware.position_command(20, -500)
        self.assertAlmostEqual(firmware.tick(), 1, places=6)
        self.assertAlmostEqual(firmware.value("debug"), cogging, places=6)
        cogging = 0.8
        firmware.position_command(0, 800)
        self.assertAlmostEqual(firmware.tick(), firmware.limit, places=6)

    def test_feedforward_does_not_leak_into_current_or_velocity_modes(self):
        firmware = self.firmware
        firmware.position_command(0, 500)
        self.assertAlmostEqual(firmware.tick(), 0.5, places=6)
        firmware.command(0x01, struct.pack(">i", 300))
        self.assertAlmostEqual(firmware.tick(), 0.3, places=6)
        firmware.command(0x03, struct.pack(">i", 0))
        self.assertEqual(firmware.tick(), 0)
        firmware.position_command(0, 0)
        self.assertEqual(firmware.tick(), 0)

    def test_complete_command_is_published_with_interrupts_masked(self):
        firmware = self.firmware
        writes = []

        def on_write(cpu, access, address, size, value, context):
            writes.append((address, cpu.reg_read(UC_ARM_REG_PRIMASK)))

        fields = [firmware.fields[name] for name in ("theta_ref", "i_ff", "CONTROL_MODE")]
        for address in fields:
            firmware.cpu.hook_add(UC_HOOK_MEM_WRITE, on_write, begin=address, end=address + 3)
        for previous_mask in (0, 1):
            writes.clear()
            firmware.cpu.reg_write(UC_ARM_REG_PRIMASK, previous_mask)
            firmware.position_command(12.34, -50)
            self.assertEqual({address for address, mask in writes}, set(fields))
            self.assertTrue(all(mask == 1 for address, mask in writes))
            self.assertEqual(firmware.cpu.reg_read(UC_ARM_REG_PRIMASK), previous_mask)


if __name__ == "__main__":
    unittest.main()
