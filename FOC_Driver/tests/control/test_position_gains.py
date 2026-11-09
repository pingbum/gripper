"""Execute the built ARM firmware offline; no CAN device or MCU is opened.

Requires: python -m pip install unicorn==2.1.4 pyelftools==0.33
Run after building Debug: python tests/control/test_position_gains.py
FOC_TEST_ELF may select another ELF with its symbol table retained.

Only the HAL CAN TX and Flash write boundaries are mocked. This verifies CPU
code, not CAN timing, interrupt preemption, Flash hardware or motor stability.
"""
import os
import math
from pathlib import Path
import struct
import unittest

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_CPU_ARM_CORTEX_M4, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC,
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_S0, UC_ARM_REG_SP,
)

ROOT = Path(__file__).resolve().parents[2]
ELF_PATH = Path(os.environ.get("FOC_TEST_ELF", ROOT / "build/Debug/FOC_Driver.elf"))
PARAM_ADDR = 0x08040000
SCRATCH = 0x2001E000
RETURN_PC = 0x0807FFF0
DRIVER_ID = 2  # Slave default CAN ID (DEFAULT_DRIVER_ID in can.cpp).


class Firmware:
    def __init__(self):
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        self.cpu.mem_map(0x08000000, 0x80000)
        self.cpu.mem_map(0x20000000, 0x20000)
        with ELF_PATH.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name: s["st_value"]
                            for s in elf.get_section_by_name(".symtab").iter_symbols()}
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    self.cpu.mem_write(segment["p_vaddr"], segment.data())
        self.cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)
        self.cpu.reg_write(UC_ARM_REG_FPEXC, 1 << 30)
        self.cpu.mem_write(PARAM_ADDR, bytes([0xFF]) * 48)
        self.tx = []
        self.flash_writes = []
        self.flash_status = 0
        self.corrupt_flash = False
        self.hook("HAL_FDCAN_AddMessageToTxFifoQ", self.can_tx)
        self.hook("write_flash_buffer", self.flash_write)
        # Normal startup runs this C++ initializer to compute alpha/beta.
        # Do not run other constructors that initialize board peripherals.
        self.call("_GLOBAL__sub_I__Z18velocity_loop_initv")
        self.call("_Z18position_loop_initv")

    def hook(self, symbol, callback):
        address = self.symbols[symbol] & ~1
        self.cpu.hook_add(UC_HOOK_CODE, callback, begin=address, end=address)

    def return_from_hal(self, result):
        self.cpu.reg_write(UC_ARM_REG_R0, result)
        self.cpu.reg_write(UC_ARM_REG_PC, self.cpu.reg_read(UC_ARM_REG_LR))

    def can_tx(self, cpu, address, size, context):
        header = struct.unpack("<9I", cpu.mem_read(cpu.reg_read(UC_ARM_REG_R1), 36))
        payload = bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_R2), header[3]))
        self.tx.append((header, payload))
        self.return_from_hal(0)

    def flash_write(self, cpu, address, size, context):
        destination, source, count, element_size = (
            cpu.reg_read(reg) for reg in (UC_ARM_REG_R0, UC_ARM_REG_R1,
                                         UC_ARM_REG_R2, UC_ARM_REG_R3))
        payload = bytes(cpu.mem_read(source, count * element_size))
        self.flash_writes.append((destination, payload))
        if self.flash_status == 0:
            cpu.mem_write(destination, payload)
            if self.corrupt_flash:
                cpu.mem_write(destination + 28, b"\xFF" * 4)
        self.return_from_hal(self.flash_status)

    def call(self, symbol, *arguments, floating=None):
        self.cpu.reg_write(UC_ARM_REG_SP, 0x2001FF00)
        self.cpu.reg_write(UC_ARM_REG_LR, RETURN_PC | 1)
        for register, argument in zip((UC_ARM_REG_R0, UC_ARM_REG_R1,
                                       UC_ARM_REG_R2, UC_ARM_REG_R3), arguments):
            self.cpu.reg_write(register, argument)
        if floating is not None:
            self.cpu.reg_write(UC_ARM_REG_S0,
                               struct.unpack("<I", struct.pack("<f", floating))[0])
        self.cpu.emu_start(self.symbols[symbol] | 1, RETURN_PC, count=500000)
        if self.cpu.reg_read(UC_ARM_REG_PC) != RETURN_PC:
            raise RuntimeError(f"{symbol} did not return within the instruction limit")
        return self.cpu.reg_read(UC_ARM_REG_R0)

    def gains(self, position=True):
        symbol = "_Z23position_loop_get_gainsPfS_" if position else "_Z23velocity_loop_get_gainsPfS_"
        self.call(symbol, SCRATCH + 128, SCRATCH + 132)
        return struct.unpack("<ff", self.cpu.mem_read(SCRATCH + 128, 8))

    def command(self, mode, payload=b"", driver_id=DRIVER_ID):
        # Header layout and constants from STM32G4 HAL FDCAN.
        header = struct.pack("<10I", (mode << 8) | driver_id,
                             0x40000000, 0, len(payload), 0, 0, 0x200000, 0, 0, 0)
        self.cpu.mem_write(SCRATCH, header)
        self.cpu.mem_write(SCRATCH + 64, payload or b"\0")
        self.call("_ZN12_GLOBAL__N_1L17handle_rx_messageERK21FDCAN_RxHeaderTypeDefPKh",
                  SCRATCH, SCRATCH + 64)

    def position_output(self, theta, target, rev=0):
        # position_loop reads only the first four fields of position_instance.
        self.cpu.mem_write(SCRATCH, struct.pack("<fffi", theta, 0, theta, rev))
        self.call("_Z13position_loopP23position_instance_f32_tf", SCRATCH, floating=target)
        return struct.unpack("<f", struct.pack("<I", self.cpu.reg_read(UC_ARM_REG_S0)))[0]


class PositionGainTests(unittest.TestCase):
    def setUp(self):
        self.firmware = Firmware()

    def test_initial_gains_and_direct_can_write_readback(self):
        firmware = self.firmware
        self.assertTrue(all(math.isfinite(gain) and gain >= 0
                            for gain in firmware.gains()))
        velocity_before = firmware.gains(position=False)
        for write_mode, read_mode, value, padding in (
                (0x22, 0xA2, 0.0125, b""), (0x23, 0xA3, 0.001, bytes(4))):
            firmware.command(write_mode, struct.pack(">f", value) + padding)
            self.assertEqual(len(firmware.tx), 0)  # No write ACK.
            firmware.command(read_mode, bytes(8))
            header, payload = firmware.tx.pop()
            self.assertEqual(header[0:4], ((read_mode << 8) | DRIVER_ID, 0x40000000, 0, 4))
            self.assertEqual(header[5:7], (0x100000, 0x200000))  # FD+BRS ON.
            self.assertAlmostEqual(struct.unpack(">f", payload)[0], value)
        self.assertEqual(firmware.gains(position=False), velocity_before)

    def test_reject_invalid_gain_payload_and_other_driver(self):
        firmware = self.firmware
        initial = firmware.gains()
        for mode in (0x22, 0x23):
            for value in (-1, float("nan"), float("inf"), -float("inf")):
                firmware.command(mode, struct.pack(">f", value))
                self.assertEqual(firmware.gains(), initial)
            for payload in (b"", b"\x01", bytes(2)):
                firmware.command(mode, payload)
                self.assertEqual(firmware.gains(), initial)
            firmware.command(mode, struct.pack(">f", 1), driver_id=1)
            self.assertEqual(firmware.gains(), initial)

    def test_gain_setters_preserve_derivative_state(self):
        firmware = self.firmware
        firmware.position_output(0.01, 0.01)
        state_address = firmware.symbols["_ZL16position_control"]
        old_state = firmware.cpu.mem_read(state_address, 16)
        self.assertNotEqual(bytes(old_state), bytes(16))
        firmware.command(0x22, struct.pack(">f", 0.1))
        firmware.command(0x23, struct.pack(">f", 0.005))
        self.assertEqual(firmware.cpu.mem_read(state_address, 16), old_state)

    def test_updated_position_gain_drives_unclamped_pd_output(self):
        firmware = self.firmware
        firmware.command(0x22, struct.pack(">f", 0.1))
        firmware.command(0x23, struct.pack(">f", 0))
        self.assertAlmostEqual(firmware.position_output(0, 1), 0.1)
        # TIM7 now adds feedforward/cogging before limiting the final reference.
        self.assertAlmostEqual(firmware.position_output(0, 1000), 100)
        self.assertAlmostEqual(firmware.position_output(0, -1000), -100)
        # Multi-turn position is not wrapped to a shortest-path angle.
        self.assertAlmostEqual(firmware.position_output(0, 361, rev=1), 0.1)

    def test_updated_derivative_gain_uses_existing_100hz_filter(self):
        firmware = self.firmware
        firmware.command(0x23, struct.pack(">f", 0.001))
        frequency = 100 * 2 * math.pi
        alpha = (10000 - frequency) / (10000 + frequency)
        beta = frequency * 10000 / (10000 + frequency)
        first = firmware.position_output(0.1, 0.1)  # P error is zero.
        self.assertAlmostEqual(first, -0.001 * beta * 0.1, places=6)
        self.assertAlmostEqual(firmware.position_output(0.1, 0.1),
                               first * alpha, places=6)

    def test_legacy_flash_load_retains_position_defaults(self):
        firmware = self.firmware
        # Legacy block: magic, cutoff, velocity Kp/Ki, rate, old tail padding.
        legacy = struct.pack("<IfffII", 0x4350524D, 1800, 0.001, 0.000002, 100, 0)
        firmware.cpu.mem_write(PARAM_ADDR, legacy)
        defaults = firmware.gains()
        self.assertEqual(firmware.call("_Z30load_control_params_from_flashv"), 1)
        self.assertEqual(firmware.gains(), defaults)
        kp, ki = firmware.gains(position=False)
        self.assertAlmostEqual(kp, 0.001)
        self.assertAlmostEqual(ki, 0.000002)

    def test_save_and_reload_position_extension(self):
        firmware = self.firmware
        firmware.command(0x22, struct.pack(">f", 0.0125))
        firmware.command(0x23, struct.pack(">f", 0.001))
        expected = firmware.gains()
        velocity = firmware.gains(position=False)
        self.assertEqual(firmware.call("_Z28save_control_params_to_flashv"), 1)
        destination, payload = firmware.flash_writes.pop()
        self.assertEqual(destination, PARAM_ADDR)
        self.assertEqual(len(payload), 48)  # Includes the slave electrical offset.
        self.assertEqual(struct.unpack_from("<I", payload, 0)[0], 0x4350524D)
        self.assertEqual(struct.unpack_from("<ff", payload, 8), velocity)
        self.assertEqual(struct.unpack_from("<I", payload, 24)[0], 0x50504431)
        self.assertEqual(struct.unpack_from("<ff", payload, 28), expected)
        firmware.command(0x22, struct.pack(">f", 0))
        firmware.command(0x23, struct.pack(">f", 0))
        self.assertEqual(firmware.gains(), (0, 0))
        self.assertEqual(firmware.call("_Z30load_control_params_from_flashv"), 1)
        self.assertEqual(firmware.gains(), expected)

    def test_flash_write_failure_and_readback_mismatch(self):
        firmware = self.firmware
        firmware.flash_status = 1
        self.assertEqual(firmware.call("_Z28save_control_params_to_flashv"), 0)
        firmware.flash_status = 0
        firmware.corrupt_flash = True
        self.assertEqual(firmware.call("_Z28save_control_params_to_flashv"), 0)


if __name__ == "__main__":
    unittest.main()
