# Orin NX CAN0 TDC candidate

`orin_can0_tdc_overlay.dts` adds only the CAN0 `prod-settings` recommended by
[NVIDIA support](https://forums.developer.nvidia.com/t/canfd-transmission-failure-at-5mbps-on-orin-nx-16gb-module-tdc-offset-issue/367473/5).
Its target `/bus@0/mttcan@c310000` was verified on the inspected Orin NX running
L4T 36.4.7, where that node has no `prod-settings`.

The source compiled successfully and merged into a copy of the live device tree
with `fdtoverlay`. The merged 5M setting was checked as `0 48 7f00 600`.
This validates the overlay structure and target; hardware operation has not been
verified. No boot configuration or live device tree has been changed.

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
