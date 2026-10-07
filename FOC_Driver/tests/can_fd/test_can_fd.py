"""No hardware is opened: source-contract checks and fake-bus probe tests.

Firmware compilation and on-wire tests are separate; source checks are not an
emulation of FDCAN, interrupts, transceiver delay or stack corruption.
"""
from collections import deque
from pathlib import Path
import re
import sys
import unittest
from unittest.mock import patch

import can

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import can_fd_check as check


class FirmwareContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "Application/Communication/Src/can.cpp").read_text()

    def test_generated_timing_matches_ioc(self):
        source = (ROOT / "Core/Src/fdcan.c").read_text()
        ioc = (ROOT / "FOC_Driver.ioc").read_text()
        expected = dict(NominalPrescaler=2, NominalSyncJumpWidth=8,
                        NominalTimeSeg1=67, NominalTimeSeg2=17,
                        DataPrescaler=2, DataSyncJumpWidth=4,
                        DataTimeSeg1=12, DataTimeSeg2=4)
        for name, value in expected.items():
            self.assertIn(f"hfdcan1.Init.{name} = {value};", source)
            self.assertRegex(ioc, rf"(?m)^FDCAN1\.{name}={value}$")
        self.assertIn("RCC.FDCANFreq_Value=170000000", ioc)
        self.assertIn("hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;", source)
        self.assertEqual(170_000_000 // (2 * (1 + 67 + 17)), 1_000_000)
        self.assertEqual(170_000_000 // (2 * (1 + 12 + 4)), 5_000_000)

    def test_both_tx_paths_are_fd_brs(self):
        self.assertEqual(self.source.count(".BitRateSwitch = FDCAN_BRS_ON"), 2)
        self.assertEqual(self.source.count(".FDFormat = FDCAN_FD_CAN"), 2)
        status = self.source.split("void CAN_Handler::broadcast_motor_status", 1)[1]
        self.assertIn(".DataLength = FDCAN_DLC_BYTES_8", status)
        self.assertIn("uint8_t tx_data[8];", status)
        self.assertIn("tx_data[6] = rev;", status)
        self.assertIn("tx_data[7] = error_code;", status)

    def test_setup_precedes_start(self):
        setup = self.source.split("void CAN_Handler::FDCAN1_SetupFiltersAndStart", 1)[1]
        setup = setup.split('extern "C" void HAL_FDCAN_RxFifo1Callback', 1)[0]
        for call in ("ConfigTxDelayCompensation", "EnableTxDelayCompensation",
                     "ConfigFilter", "ConfigGlobalFilter", "ActivateNotification"):
            self.assertLess(setup.index("HAL_FDCAN_" + call),
                            setup.index("HAL_FDCAN_Start"))
        self.assertIn("hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler", setup)
        self.assertIn("tdc_offset > 127U", setup)
        self.assertIn("FDCAN_REJECT_REMOTE", setup)

    def test_rx_scratch_and_length_guard_before_dispatch(self):
        rx = self.source.split('extern "C" void HAL_FDCAN_RxFifo1Callback', 1)[1]
        rx = rx.split("void CAN_Handler::broadcast_motor_status", 1)[0]
        self.assertIn("uint8_t data[64] = {};", rx)
        guard = rx[rx.index("if (rxh.RxFrameType"):rx.index("handle_rx_message(rxh, data)")]
        self.assertEqual(re.findall(r"rxh.DataLength != FDCAN_DLC_BYTES_(\d+)", guard),
                         ["0", "1", "4", "8"])
        self.assertIn("continue;", guard)


class ProbeTests(unittest.TestCase):
    def message(self, eid=0x8701, data=b"\x01", **options):
        defaults = dict(is_extended_id=True, is_fd=True, bitrate_switch=True, is_rx=True)
        defaults.update(options)
        return can.Message(arbitration_id=eid, data=data, **defaults)

    def test_read_request_has_no_write_or_motion_mode(self):
        for driver_id in (0, 1, 255):
            msg = check.read_request(driver_id)
            self.assertEqual(msg.arbitration_id, 0x8700 | driver_id)
            self.assertEqual(bytes(msg.data), bytes(8))
            self.assertTrue(msg.is_fd and msg.bitrate_switch and msg.is_extended_id)

    def test_valid_reply_and_status(self):
        self.assertEqual(check.classify(self.message(), [0, 1]), ("reply", 1, b"\x01"))
        self.assertEqual(check.classify(self.message(eid=0, data=bytes(8)), [0, 1]),
                         ("status", 0, bytes(8)))

    def test_reject_wrong_flags_echo_id_and_length(self):
        variants = [dict(is_fd=False), dict(bitrate_switch=False), dict(is_rx=False),
                    dict(is_extended_id=False), dict(is_remote_frame=True),
                    dict(is_error_frame=True), dict(eid=0x8702, data=b"\x02"),
                    dict(data=b"\x00"), dict(data=bytes(8)),
                    dict(eid=1, data=bytes(64))]
        for options in variants:
            with self.subTest(options=options):
                self.assertIsNone(check.classify(self.message(**options), [0, 1]))

    def test_slcan_timing_without_serial_baud_setting(self):
        options = check.connection_options("slcan", "COM9")
        self.assertEqual(options["timing"].nom_bitrate, 1_000_000)
        self.assertEqual(options["timing"].data_bitrate, 5_000_000)
        self.assertNotIn("tty_baudrate", options)

    def test_socketcan_fd_option(self):
        with patch.object(check.sys, "platform", "linux"):
            options = check.connection_options("socketcan", "can0")
        self.assertTrue(options["fd"])
        self.assertFalse(options["receive_own_messages"])
        self.assertNotIn("bitrate", options)

    def test_probe_fake_bus_and_no_response(self):
        class FakeBus:
            def __init__(self, respond):
                self.messages, self.sent, self.respond = deque(), [], respond

            def send(self, msg, timeout):
                self.sent.append(msg)
                if self.respond:
                    driver_id = msg.arbitration_id & 0xFF
                    self.messages.extend([
                        can.Message(arbitration_id=msg.arbitration_id, data=[driver_id],
                                    is_extended_id=True, is_fd=True, bitrate_switch=True),
                        can.Message(arbitration_id=driver_id, data=bytes(8),
                                    is_extended_id=True, is_fd=True, bitrate_switch=True)])

            def recv(self, timeout):
                return self.messages.popleft() if self.messages else None

        for respond in (False, True):
            bus = FakeBus(respond)
            ticks = iter(i * 0.001 for i in range(10000))
            with patch.object(check.time, "monotonic", side_effect=lambda: next(ticks)):
                results, errors = check.probe(bus, [0, 1], 0.05)
            self.assertEqual(errors, 0)
            self.assertEqual(len(bus.sent), 2)
            self.assertTrue(all(msg.arbitration_id >> 8 == 0x87 for msg in bus.sent))
            for result in results.values():
                self.assertEqual(result["reply"], respond)
                self.assertEqual(result["status_count"], int(respond))


if __name__ == "__main__":
    unittest.main()
