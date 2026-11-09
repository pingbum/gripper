# Position command with feedforward current

The firmware position command uses mode `0x05` and an extended CAN identifier:
`EID = (0x05 << 8) | driver_id`. Send CAN FD with BRS and exactly 8 data bytes.

| Bytes | Encoding | Meaning |
| --- | --- | --- |
| 0-3 | Signed int32, big-endian | Target output position in 0.01 degrees |
| 4-5 | Signed int16, big-endian | Feedforward q-axis current in mA |
| 6-7 | Reserved | Send zero; firmware ignores these bytes |

Encode current as `round(i_ff_A * 1000)`. The representable range is
`-32768..32767` mA (-32.768..32.767 A), with 1 mA resolution. Current uses the
same mechanical sign convention as mode `0x01`; the current loop applies the
encoder direction once. This field is an absolute current, not a percentage
of `CURRENT_MAX` and not torque in Nm.

Example: driver 1, target +30 degrees, feedforward +0.5 A:

```text
Extended ID: 0x0501
Data:        00 00 0B B8 01 F4 00 00
```

For -0.5 A, replace bytes 4-5 with `FE 0C`. An 8-byte position command with
zero padding still requests zero feedforward. Legacy 4-byte position commands
are rejected without changing the current command. Other command lengths and
IDs are unchanged. Position writes do not produce a separate acknowledgement.

Position and feedforward are published together and held until the next valid
position command. Feedforward starts at zero, is used only in position mode,
and is not saved to Flash. Every 200 us, the firmware computes:

```text
iq_ref = clamp(position_PD + i_ff + cogging_current,
               -CURRENT_MAX, +CURRENT_MAX)
```

The PD output and feedforward are not separately clamped before addition.
Only the final limited reference is published to the 50 kHz current loop.
This bounds the commanded current; it is not an instantaneous measured-current
guarantee. The application must place the signed mA value in bytes 4-5 instead
of its previous zero padding.
