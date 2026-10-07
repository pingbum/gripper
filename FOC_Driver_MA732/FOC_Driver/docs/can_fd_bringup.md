# CAN FD bring-up: 1 Mbit/s arbitration / 5 Mbit/s data

Both firmware projects use ISO CAN FD with BRS. Motor control/status payloads
remain 8 bytes with the same IDs, byte order, scales and field meanings.
Existing management responses remain 1 or 4 bytes. Flash-stored driver IDs are
unchanged (firmware defaults: Main 0, Slave 1). SPI and motor-control code are not
changed. `tools/app.py` also uses FD+BRS for every TX path and starts RX-only
on Connect; see [monitor setup](../tools/monitor_setup.md).

## Firmware

- FDCAN kernel clock: PCLK1, 170 MHz, divider 1.
- Nominal: prescaler 2, seg1 67, seg2 17, SJW 8 -> 1M, sample point 80%.
- Data: prescaler 2, seg1 12, seg2 4, SJW 4 -> 5M, sample point 76.47%.
- TDC: offset = DataTimeSeg1 * DataPrescaler = 24 kernel-clock periods,
  filter = 0, enabled before start. This follows the initial setting recommended
  in [ST's G4 example](https://github.com/STMicroelectronics/STM32CubeG4/blob/master/Projects/STM32G474E-EVAL/Examples/FDCAN/FDCAN_Com_polling/Src/main.c).
  Physical timing still needs validation on the actual board/transceiver.
- Filters, remote-frame rejection and FIFO1 RX notification are configured before
  start. Non-matching frames no longer accumulate in the unused FIFO0.
- HAL RX scratch buffer is 64 bytes; software accepts only existing DLC 0/1/4/8
  data frames before dispatching any command. Larger frames are drained/rejected.
  Classic CAN RX remains accepted for compatibility; every firmware TX is FD+BRS.

Debugger counters: `can_status_tx_queued`, `can_status_tx_errors`, `can_rx_frames`,
`can_rx_fd_brs_frames`, `can_rx_rejected_frames`. Queued TX is NOT proof of ACK.
Inspect FDCAN ECR/PSR for protocol/error counters; the two rejection/RX counters
are diagnostic and include frames addressed to other nodes.

## Before connecting

1. Build and flash the corresponding firmware into each board. Do not change IDs
   or overwrite calibration/parameter flash. This guide does not flash hardware.
2. Confirm the board transceiver AND the PC adapter/firmware support ISO FD at 5M.
   Generic Classic-CAN-only SLCAN adapters cannot be used. Do not leave an active
   Classic-only node on the FD bus.
   Adafruit CAN Pal takes 3.3V power with a 3.3V host: its onboard charge pump
   generates the TJA1051's 5V supply. Do not apply the bare-chip VCC requirement
   to the module's external Vcc header; follow the
   [module guide](https://learn.adafruit.com/adafruit-can-pal/pinouts).
3. Power off before checking termination/wiring. Use the two bus-end 120-ohm
   terminators (about 60 ohms in parallel), common ground, short stubs and twisted
   CAN-H/CAN-L. A 60-ohm reading alone does not validate 5M signal integrity.
4. Keep the mechanism secured and motor drive disabled for first communication
   tests. The check script never requests motion, but does not disable motor
   drive or override the board's normal startup behaviour.
5. Close app.py and other CAN tools. Main must be powered for Slave's shared-SPI
   encoder data to be valid. First test Main alone with `--ids 0`, then both.

## Standalone PC check

Run from `FOC_Driver` using Python with `python-can==4.6.1` and `pyserial` installed
(the existing GUI environment is suitable).

### Windows / compatible SLCAN FD

Replace COM9 with your adapter port:

```powershell
python tools/can_fd_check.py --interface slcan --channel COM9 --ids 0 1
```

This uses `BitTimingFd` so python-can sends the CANable 2.0-compatible S8/Y5
bitrate commands. An FD-capable chip alone is insufficient: the adapter firmware
must implement that protocol. No Serial baud UI/setting is added.
[python-can SLCAN details](https://python-can.readthedocs.io/en/v4.6.1/interfaces/slcan.html)

### Linux / native FD-capable SocketCAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
ip -details -statistics link show can0
python tools/can_fd_check.py --interface socketcan --channel can0 --ids 0 1
```

Check FD is enabled, rates match and error counters do not grow. `slcand` does
not make a Classic-only adapter/driver FD-capable. An alternative on Linux is
direct compatible SLCAN with `--interface slcan --channel /dev/ttyACM0`.
[SocketCAN configuration](https://docs.kernel.org/networking/can.html)

## What PASS means

The script sends only an 8-byte zero-filled driver-ID **read** request using EID
`0x8700 | driver_id`, FD=1, BRS=1. There are no motion, calibration, reset or flash
write commands. It retries unanswered reads every 0.5 seconds for 5 seconds.
Use `--once` to submit each ID only once (Linux/controller retries may still
occur). The probe stops submitting requests if a BUS-OFF error frame arrives.

- Driver 0 request: extended `00008700`, data `00 00 00 00 00 00 00 00`.
- Driver 1 request: extended `00008701`, same data.
- Replies: same EID, one byte `00` / `01` (existing protocol), FD=1, BRS=1.
- Status: extended EID `00000000` / `00000001`, 8 bytes, FD=1, BRS=1.

Each ID passes only after both its matching read response and its 8-byte status
are received as FD+BRS. This verifies a bidirectional application exchange, not
just a successful USB write or a local TX echo. It does not certify 5M signal
integrity or loss-free full-rate operation. Error-frame reporting depends on
the backend. A nonzero motor error byte is reported separately even if transport
passes. If status broadcasting was previously disabled, a read reply alone does
not pass the status check; the script deliberately does not change settings.

### RX works but Jetson TX enters BUS-OFF

Close the GUI and test one driver-ID read. Keep the current 1M/5M interface
configuration; no firmware change is needed for this comparison:

```bash
python3 tools/can_fd_check.py --interface socketcan --channel can0 --ids 0 --once --seconds 2
```

After recovering `can0` with the same timing if the first test entered BUS-OFF,
compare an FD request without bitrate switching:

```bash
python3 tools/can_fd_check.py --interface socketcan --channel can0 --ids 0 --once --no-brs --seconds 2
```

This is still CAN FD, not Classic CAN. Only the request uses nominal-rate data;
the existing firmware's read reply/status remain FD+BRS. If only the no-BRS
request gets a matching reply, investigate Jetson data-phase TX timing/TDC and
signal quality first. If neither works, check TX pinmux, TX wiring, transceiver
silent mode, ACK/error frames and both endpoints' timing. `TX submissions` only
counts backend acceptance, while `read reply=True` proves the application exchange.
Neither outcome alone proves which component is faulty. NVIDIA documents
[Jetson pinmux and TDCR](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/HR/ControllerAreaNetworkCan.html).

If the adapter connects but no frames arrive, check adapter firmware first,
then nominal/data timing, BRS, wiring, board ECR/PSR and TDC. If only the read
reply arrives, inspect broadcast enable/rate and the firmware status timer.

## Offline regression checks

```powershell
python -B -m unittest discover -s tests/can_fd -p "test_*.py"
```

These use source-contract checks and a fake bus. They neither open hardware nor
replace the real-board FD test. Build both firmware projects separately.
