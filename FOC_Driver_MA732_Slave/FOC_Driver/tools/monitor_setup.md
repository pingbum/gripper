# MA732 monitor: Windows / Linux

Master and Slave use the same `tools/app.py` and GUI modules. Select the board
with **Driver ID** (TX) and **Listen Driver ID** (plot). Run only one monitor per
serial adapter. This change does not alter firmware or the CAN packet layout.

## Install and run

Python **3.11 or newer** is recommended (Windows high-resolution sleep support).
From `FOC_Driver`, create a separate virtual environment on each OS. Do not copy
Windows `.venv` or build products to Linux.

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install -r tools/requirements.txt
.\.venv\Scripts\python tools/app.py
```

Linux:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/requirements.txt
.venv/bin/python tools/app.py
```

A desktop session and the Qt/X11 runtime libraries for your distribution are
required. Use `QT_QPA_PLATFORM=offscreen` only for automated tests.

## Adapter settings

| Adapter / OS | Interface | Channel | Bit timing |
| --- | --- | --- | --- |
| Windows, CANable 2.0-compatible FD firmware | `slcan` | actual `COM` port | fixed 1M/5M via FD timing |
| Linux, native FD-capable SocketCAN | `socketcan` | `can0` etc. | configure with `ip link` first |
| Linux, CANable 2.0-compatible FD firmware | `slcan` | `/dev/ttyACM0` or `/dev/ttyUSB0` | fixed 1M/5M via FD timing |

The app defaults to `slcan` on Windows and `socketcan` on Linux. **Refresh ports**
lists available channels; custom names remain editable. Successful connection
settings are remembered per OS and backend. SocketCAN disables the bitrate field
because the GUI does not change the kernel interface settings.

Every app TX path (scan, reads, writes, RAW CAN and reference generator) uses
extended **ISO CAN FD**, including short payloads. **TX BRS (5M)** selects
bitrate switching for every manual/scan/parameter/reference command. SocketCAN
defaults to BRS **off**: requests use 1M for the data phase while FD+BRS RX stays
enabled at 1M/5M. Compatible SLCAN defaults to BRS on. The preference is saved
per interface; disconnect before changing it. Socket `fd=True` alone
does not set outgoing message flags. There is no Classic-CAN TX fallback.
The python-can `gs_usb` backend is not offered because it only supports Classic
CAN; use an FD-capable SocketCAN device or compatible SLCAN FD firmware.

For an existing native FD-capable `can0` interface at 1M/5M, with the bus idle:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on berr-reporting on restart-ms 0
sudo ip link set can0 up
ip -details -statistics link show can0
```

Connect starts **RX only**, without automatic scan or parameter requests.
Broadcast status frames discover motors in the dropdown without sending traffic.
Use **Scan Motors** for explicit driver-ID reads (IDs 0..255, 50 Hz) and
**Read Params** to fetch the selected Driver ID's BW/Kp/Ki. Changing the selected
motor does not transmit. TX remaining at 0 Hz until a request is normal.

If connecting used to stop candump, inspect `ip -details -statistics link show
can0` before resetting it. BUS-OFF stops the whole interface, not just the GUI.
BUS-OFF is fatal: the RX worker stops TX and the GUI disconnects. RX-only
error-passive (`ctrl=0x10`) warns without blocking TX; an error-passive CAN node
can still transmit. TX error-passive (`ctrl=0x20` or `0x30`) pauses TX while
RX/plots continue. Pending commands, scan/parameter retries and
the reference waveform are discarded. **Check CAN / Resume TX** runs a read-only
`ip -details -json link show dev <channel>` in a background worker. New TX requests
are enabled only when the kernel reports ERROR-ACTIVE and no new TX-passive warning
arrived during the check. This does not reset/reconfigure the interface, send
test packets or restart an old waveform. A stopped/failing worker still requires
reconnection; SLCAN recovery requires reconnection as no kernel CAN state exists.
Repeated counter-byte changes in the same passive state count toward **warnings**
but do not flood the log. State changes/new pause episodes are logged, and an
ERROR-ACTIVE recovery notification is shown if the driver emits it. A successful
manual state check works even when the driver omits that recovery notification.
The log distinguishes RX/TX passive flags and retains the first raw error data.
The TX-ready label means that the app accepts requests, not that hardware TX
or bus ACK has been verified. A `Calibration queued` followed by BUS-OFF is a
transport failure; use a driver-ID read for diagnosis instead of resetting or
calibrating the motor again.
Driver error reporting is needed;
absence of an error frame is not proof that the interface is healthy. Already
queued controller transmissions can still retry; the app does not reset the
interface or automatically replay requests. ERROR-ACTIVE at one instant does not
prove error-free traffic; watch cumulative RX errors over time. The GUI TX rate counts backend
submissions, not kernel TX success or motor ACKs.

SLCAN uses python-can's default serial-port settings and `BitTimingFd` to send
1M/5M selectors to compatible FD firmware. Generic SLCAN/slcand and Classic-only
adapters do not provide FD support. Serial-device permissions on Linux depend on the
distribution (often the `dialout` group); configure access rather than running
the GUI as root.

At 5M, successful RX alone does not validate Jetson TX timing or transceiver
delay compensation. If a single explicit FD+BRS read still causes BUS-OFF,
check ACK errors, nominal/data timing, transceiver wiring and Jetson TDC.
See [NVIDIA's CAN/TDC guide](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/HR/ControllerAreaNetworkCan.html#obtaining-higher-bit-rates)
and [firmware timing/standalone FD check](../docs/can_fd_bringup.md).

On Jetson L4T versions exposing `tdc_offset`, check:

```bash
cat /sys/class/net/can0/tdc_offset
```

`tdc_offset=0x0, DBTP.tdc=0` means the driver's TDC enable flag is off. The app
logs a setup warning for this condition but does not change kernel settings.
Close the GUI, then test NVIDIA's suggested starting value (requires sudo):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.800 sjw 4 dbitrate 5000000 dsample-point 0.800 dsjw 2 fd on berr-reporting on restart-ms 0
printf '0x600\n' | sudo tee /sys/class/net/can0/tdc_offset
sudo ip link set can0 up
cat /sys/class/net/can0/tdc_offset
python3 tools/can_fd_check.py --interface socketcan --channel can0 --ids 0 --once --seconds 2
```

Run the Python command from the repository root. Check the read reply and kernel
TX counters before trying Calibration again. `0x600` is a starting point, not a
validated value for every transceiver/board. A sysfs readback of `DBTP.tdc=1`
does not by itself verify hardware TDCR programming. NVIDIA has investigated
[5M failures with missing mttcan prod-settings](https://forums.developer.nvidia.com/t/canfd-transmission-failure-at-5mbps-on-orin-nx-16gb-module-tdc-offset-issue/367473/5).
If the read still fails, compare `--no-brs` as described in the bring-up guide
and inspect the Jetson device tree/TDCR rather than assuming the motor firmware
rejects FD. The loaded CAN node on the inspected Orin NX lacked `prod-settings`;
no device tree, registers, firmware or kernel settings were changed by this check.

On that board, two FD reads with BRS off received matching FD+BRS replies,
while BRS-on requests caused BUS-OFF. The app's BRS-off default uses the working
request format without requiring a motor firmware change. Receive stuff/form
errors remained, so this is not a claim of error-free operation or a fix to 5M
Jetson TX. A [candidate CAN0 device-tree overlay](jetson/orin_can0_tdc_overlay.dts)
adds NVIDIA's missing prod settings. It was compiled and merged into a copy of
the live DT for validation; it has not been installed or applied to the system.

## Timing and recording

- **TX Hz (target)** accepts 1..1000 Hz. A single worker owns CAN transmission;
  Qt callbacks queue manual commands without waiting for the adapter. The
  reference generator computes each packet in the worker using a monotonic
  clock. Amplitude/frequency changes are copied from the GUI as a snapshot.
- Missed periodic deadlines are skipped, not replayed in a burst. **TX skipped**
  reports missed whole periods, including slow sends. OS scheduling, USB and
  bus load still affect the actual rate; this is not a real-time 1 kHz guarantee.
  Scan/read/manual commands are additional traffic beyond the reference rate.
- **Stop Ref** stops further periodic submissions. A packet already submitted
  to the backend can finish; it does not send a zero command or stop the motor.
- **TX** counts successful backend sends, not acknowledgements from motors.
  **RX** counts accepted extended frames across all IDs, not the displayed ID
  alone. A send/receive error stops transmission and disconnects; there is no
  automatic replay/reconnect. A full command queue rejects the new command.
- The plot polls the latest state of the selected ID at **40 Hz**. Other IDs
  cannot overwrite it. Plot samples are intentionally reduced; narrow peaks
  between refreshes are not preserved. Use recording for analysis.
- **Record RX CSV** captures every frame delivered by the CAN backend, across
  all IDs, before GUI filtering/reduction. It writes raw timestamps, host
  monotonic arrival time, EID, flags, DLC and hex payload on a separate thread.
  It does not log PC TX. A 10000-frame queue absorbs disk stalls; overflow is
  explicitly counted as **dropped**. It cannot detect frames lost upstream in
  the adapter/driver. Stop/Disconnect flushes pending records. Disk/open errors
  are shown in the log; existing CSV files are never overwritten.
- Display state is cleared when changing the selected ID or reconnecting.
  A selected ID with no fresh frame for 0.5 seconds is marked stale.
- Configuration/USB startup can take time during Connect; steady-state TX/RX,
  recording and disconnect cleanup run outside the GUI thread.

Future packed control packets can supply a callable to
`CANWriterThread.set_periodic(factory, rate_hz)`. The factory receives elapsed
seconds and returns `(extended_id, payload_bytes, plot_value)`. The current
reference implementation retains the existing single-driver protocol.

## Offline verification

No motor or adapter is required. Tests use fake backends and temporary CSVs.

```powershell
$env:QT_QPA_PLATFORM = 'offscreen'
.\.venv\Scripts\python -B -m unittest discover -s tests/can_monitor -p 'test_*.py'
```

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -B -m unittest discover -s tests/can_monitor -p 'test_*.py'
```

References: [python-can SLCAN](https://python-can.readthedocs.io/en/stable/interfaces/slcan.html),
[SocketCAN](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html),
[Python sleep timing](https://docs.python.org/3/library/time.html#time.sleep).
