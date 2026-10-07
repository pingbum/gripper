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
| Windows, SLCAN firmware | `slcan` | actual `COM` port | GUI CAN bitrate |
| Linux, native SocketCAN / candleLight | `socketcan` | `can0` etc. | configure with `ip link` first |
| Linux, SLCAN firmware | `slcan` | `/dev/ttyACM0` or `/dev/ttyUSB0` | GUI CAN bitrate |
| Linux, SLCAN through slcand | `socketcan` | `slcan0` etc. | configure with slcand first |

The app defaults to `slcan` on Windows and `socketcan` on Linux. **Refresh ports**
lists available channels; custom names remain editable. Successful connection
settings are remembered per OS and backend. SocketCAN disables the bitrate field
because the GUI does not change the kernel interface settings.

For an existing native `can0` interface at 1 Mbit/s, with the bus idle:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

An adapter that appears as a serial port does not automatically become `can0` on
Linux. Use `slcan` directly or set up `slcand` from `can-utils`. For example,
for an adapter configured for 1 Mbit/s CAN:

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0
sudo ip link set slcan0 up
```

SLCAN uses python-can's default serial-port settings. The app only configures
the CAN bitrate and channel. Serial-device permissions on Linux depend on the
distribution (often the `dialout` group); configure access rather than running
the GUI as root.

`gs_usb` is still available for compatible firmware. Install its optional
dependencies with `python -m pip install "python-can[gs_usb]==4.6.1"`. Windows
requires a suitable USB driver; Linux users can normally use native SocketCAN.
Switching a serial adapter to candleLight is a separate firmware change, not
performed by this app.

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
