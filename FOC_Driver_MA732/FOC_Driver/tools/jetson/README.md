# Orin NX CAN0 TDC candidate

`orin_can0_tdc_overlay.dts` adds only the CAN0 `prod-settings` recommended by
[NVIDIA support](https://forums.developer.nvidia.com/t/canfd-transmission-failure-at-5mbps-on-orin-nx-16gb-module-tdc-offset-issue/367473/5).
Its target `/bus@0/mttcan@c310000` was verified on the inspected Orin NX running
L4T 36.4.7, where that node has no `prod-settings`.

On 2026-10-08 the combined diagnostic reproduced the hardware mismatch:

```text
sysfs: tdc_offset=0x600, DBTP.tdc=1
HW DBTP @0x0c31000c = 0x00800611 (TDC=1)
HW TDCR @0x0c310048 = 0x00000000 (TDCO=0, TDCF=0)
```

These values were identical before and after one BRS-on ID read. The interface
started ERROR-ACTIVE (TX/RX counters 0), then reached BUS-OFF (TX=248, RX=0), with
no successful TX packets or read reply. This confirms that sysfs did not program
the requested TDCO; it does not yet prove that applying this overlay alone fixes
all TX or residual RX errors. The original report was
`/tmp/ma732-can5m-c3L8Q8/report.txt` (temporary; may disappear after reboot).

The source compiled successfully and merged into a copy of the live device tree
with `fdtoverlay`. The merged 5M setting was checked as `0 48 7f00 600`.
This validates the overlay structure and target; hardware operation still needs
post-boot verification. The first installed configuration used `OVERLAYS`
without `FDT`; after reboot the live tree still lacked `prod-settings` and TDCR
remained zero (`/tmp/ma732-can5m-4jQIyZ/report.txt`). The installer now includes
both fields, matching the installed NVIDIA Jetson-IO implementation and
[NVIDIA guidance for R36](https://forums.developer.nvidia.com/t/how-to-flash-dtb-only-not-the-whole-image/327420).

After the corrected FDT+OVERLAYS boot on 2026-10-08, the live tree included the
new settings and `/proc/cmdline` contained `ma732.can0_tdc=1`. Hardware TDCR then
read `0x00000600` before and after transmission. However, the BRS-on ID read
still produced bit errors and BUS-OFF with TX=248 and zero successful TX packets
(`/tmp/ma732-can5m-ZSsUtV/report.txt`). The missing TDCO was a confirmed defect,
but correcting it was not sufficient to establish 5 Mbps transmission.

The subsequent one-shot comparison (`/tmp/ma732-oneshot-8pDupy/report.txt`)
confirmed hardware DAR=1 and TDCO=6. RX-only received 1994/1997 FD+BRS status
frames for IDs 0/1 in two seconds, with ten stuff-error frames. Both BRS OFF
and ON reads then failed, with no successful TX packets. Error counters
persisted across down/up (BRS ON started with TX=152 and ended with TX=176),
so this is not a comparison starting at identical zero-error counters. The
current failure can no longer be assumed to occur only at 5 Mbps.

Read-only pad inspection found CAN0_DOUT=0xc400 (NVIDIA reference matches) and
CAN0_DIN=0xc450 (reference 0xc458 adds an input pull-up). No pad registers were
modified. Physical TXD/bus/RXD behavior and transceiver mode/power still need
verification; a matching TX mux value alone does not prove a working TX wire.

The reviewed installer is restricted to this board, L4T release, overlay source,
and original boot configuration (or the exact first overlay-only candidate).
Run without options for a dry-run:

```bash
python3 tools/jetson/install_can0_tdc.py
```

After approval to change the boot configuration and reboot, install from `tools`:

```bash
sudo python3 jetson/install_can0_tdc.py --install --reboot
```

It backs up `/boot/extlinux/extlinux.conf`, installs the compiled overlay at
`/boot/dtb/ma732-orin-can0-tdc.dtbo`, and adds/updates a `can0-tdc` default entry
with `FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb` and `OVERLAYS`.
The base DTB hash is checked and overlay merging is tested against both that
file and the live tree. The original `primary kernel` entry remains available
in a 10-second boot menu. Compilation, both merge checks, and installed file
verification must pass before reboot. The installer refuses unreviewed source
or boot configuration changes. After reboot run `check_can0_5m.sh` below and
verify hardware TDCO plus the BRS-on response; installation alone is not proof
of successful communication. `/proc/cmdline` includes `ma732.can0_tdc=1` when the
updated test entry was selected. To revert, select `primary kernel` at boot and
restore the original `extlinux.conf.pre-can0-tdc-*` backup (the first backup is
the untouched original; later backups may contain the earlier test entry).

To compare sysfs with the actual controller registers, build the read-only helper
from the repository root:

```bash
cc -O2 -Wall -Wextra -Werror -o /tmp/read_can0_tdc tools/jetson/read_can0_tdc.c
cat /sys/class/net/can0/tdc_offset
sudo /tmp/read_can0_tdc
```

Run with can0 administratively UP and do not change its state during the read.
The helper verifies the CAN0 device-tree address and runtime power state, maps
`/dev/mem` read-only, and reads CREL, CCCR, DBTP, TDCR, PSR byte 2, and the
documented CAN0_DOUT/DIN pad configuration addresses. The PSR byte 2
read exposes TDCV without reading byte 0 (which clears LEC/DLEC). TDCV is the
SSP position, including measured loop delay plus TDCO, and is cleared in INIT;
a value of zero after BUS-OFF is not evidence of a zero loop delay. See the
[Bosch manual, PSR and TDC sections](https://www.bosch-semiconductors.com/media/ip_modules/pdf_2/m_can/mttcan_users_manual_v330.pdf).
The helper sends no frames. If sysfs
reports `0x600, DBTP.tdc=1`, compare against hardware TDC=1 and TDCO=6. A mismatch
is evidence to investigate driver/device-tree programming before tuning offsets.

For a combined recovery and BRS-on test, close `app.py`, then run from `tools`:

```bash
bash jetson/check_can0_5m.sh
```

This needs interactive sudo authentication. It sets CAN0 to 1M/5M, 80% nominal
and data sample points, SJW=4/DSJW=2, FD and error reporting, no automatic restart,
and sysfs TDC=0x600. It requires hardware TDC=1/TDCO=6/TDCF=0 before submitting
one driver-ID read to ID 0 with BRS ON, then reads DBTP/TDCR again. If hardware
TDC does not match, it exits without sending. No calibration, setpoint, flash, device-tree,
or raw-register writes are performed. Results are saved under
`/tmp/ma732-can5m-*/report.txt`; a failed probe can leave can0 BUS-OFF for inspection.

When TDCO is applied but BRS-on TX still fails, compare RX-only and BRS-off/on
reads without repeated hardware retransmission (no reboot):

```bash
bash jetson/check_can0_oneshot.sh
```

This requires the driver to accept `one-shot on` and hardware CCCR.DAR=1 before
any test request. It observes RX for two seconds, then performs at most three
driver-ID read submissions per BRS mode after a down/up cycle. Error counters
may persist across that cycle and must be compared using the logged snapshots.
It logs TDCV after each mode and restores normal retransmission plus CAN0 UP
on exit. Arbitration loss can cancel a one-shot request without a bus error;
absence of a reply alone is not proof of an electrical failure. Reports use
`/tmp/ma732-oneshot-*/report.txt`. If one-shot is unsupported it stops without
submitting requests; do not assume `--once` alone disables hardware retries.

Build from the repository root:

```bash
dtc -@ -I dts -O dtb -o /tmp/orin_can0_tdc.dtbo tools/jetson/orin_can0_tdc_overlay.dts
```

Installation depends on the Jetson's boot/overlay configuration and should use
a recoverable boot entry. Verify actual TDCR programming and a BRS-on driver-ID
read after applying it. A sysfs `tdc_offset=0x600, DBTP.tdc=1` readback alone
does not establish that TDCR was programmed.

Meanwhile the monitor's SocketCAN TX defaults to FD with BRS off, which received
matching driver replies on this board while retaining FD+BRS RX. Residual
receive errors remain a separate issue.
