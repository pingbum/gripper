"""Read-only motor protocol check for ISO CAN FD, 1 Mbit/s / 5 Mbit/s.

Separate from app.py. Sends ONLY the existing driver-ID read request (mode 0x87).
No setpoints, calibration, resets, configuration writes, or flash writes.
SLCAN requires CANable 2.0-compatible FD firmware; generic SLCAN is not enough.
"""
import argparse
import math
import sys
import time

import can


READ_ID_MODE = 0x87


def connection_options(interface, channel):
    options = dict(interface=interface, channel=channel, ignore_config=True)
    if interface == "socketcan":
        if not sys.platform.startswith("linux"):
            raise ValueError("SocketCAN requires Linux")
        # Configure nominal/data bitrate with ip link before opening the socket.
        options.update(fd=True, receive_own_messages=False)
    elif interface == "slcan":
        # python-can 4.6.1 uses BitTimingFd to send S8/Y5 to compatible firmware.
        # This backend sends bitrate selectors, NOT these segment values.
        options["timing"] = can.BitTimingFd(
            f_clock=80_000_000,
            nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=4,
            data_brp=1, data_tseg1=11, data_tseg2=4, data_sjw=4,
        )
    else:
        raise ValueError("Supported interfaces: socketcan, slcan")
    return options


def read_request(driver_id, brs=True):
    return can.Message(arbitration_id=(READ_ID_MODE << 8) | driver_id,
                       is_extended_id=True, is_fd=True, bitrate_switch=brs,
                       data=bytes(8), check=True)


def classify(message, driver_ids):
    """Ignore classic/no-BRS frames and local TX echoes when deciding success."""
    if (not message.is_rx or message.is_error_frame or message.is_remote_frame
            or not message.is_extended_id or not message.is_fd
            or not message.bitrate_switch):
        return None
    eid, data = message.arbitration_id, bytes(message.data)
    if eid in driver_ids and message.dlc == 8 and len(data) == 8:
        return "status", eid, data
    driver_id = eid & 0xFF
    if (eid == (READ_ID_MODE << 8) | driver_id and driver_id in driver_ids
            and message.dlc == 1 and data == bytes([driver_id])):
        return "reply", driver_id, data
    return None


def describe_error(message):
    """Linux CAN error.h classes/flags; retain raw bytes for driver specifics."""
    eid, data = message.arbitration_id, bytes(message.data)
    labels = []
    for mask, name in ((0x01, "TX timeout"), (0x20, "no ACK"),
                       (0x40, "BUS-OFF"), (0x80, "bus error")):
        if eid & mask:
            labels.append(name)
    if eid & 0x04 and len(data) > 1:
        for mask, name in ((0x04, "RX warning"), (0x08, "TX warning"),
                           (0x10, "RX passive"), (0x20, "TX passive"),
                           (0x40, "ERROR-ACTIVE")):
            if data[1] & mask:
                labels.append(name)
    if eid & 0x08 and len(data) > 3:
        for mask, name in ((0x01, "bit error"), (0x02, "form error"),
                           (0x04, "stuff error"), (0x08, "cannot send dominant bit"),
                           (0x10, "cannot send recessive bit")):
            if data[2] & mask:
                labels.append(name)
        labels.append(f"protocol location=0x{data[3]:02X}")
    return f"EID=0x{eid:08X}, {', '.join(labels) or 'unspecified'}, data={data.hex(' ')}"


def probe(bus, driver_ids, timeout, *, brs=True, once=False, on_error=None):
    # Discard previously buffered frames before sending the fresh read requests.
    drain_until = time.monotonic() + 0.1
    while time.monotonic() < drain_until:
        if bus.recv(timeout=0.0) is None:
            break
    results = {driver_id: {"reply": False, "status_count": 0, "last_status": b"",
                           "requests_sent": 0}
               for driver_id in driver_ids}
    deadline = time.monotonic() + timeout
    next_request = 0.0
    error_frames = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            for driver_id, result in results.items():
                if not result["reply"] and (not once or result["requests_sent"] == 0):
                    bus.send(read_request(driver_id, brs=brs), timeout=0.1)
                    result["requests_sent"] += 1
            next_request = now + 0.5
        msg = bus.recv(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
        if msg is None:
            continue
        if msg.is_error_frame:
            error_frames += 1
            if on_error is not None:
                on_error(msg)
            if msg.arbitration_id & 0x40:  # CAN_ERR_BUSOFF
                break
        decoded = classify(msg, results)
        if decoded is not None:
            kind, driver_id, data = decoded
            result = results[driver_id]
            if kind == "reply":
                result["reply"] = True
            else:
                result["status_count"] += 1
                result["last_status"] = data
    return results, error_frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", choices=("slcan", "socketcan"),
                        default="socketcan" if sys.platform.startswith("linux") else "slcan")
    parser.add_argument("--channel", required=True, help="COM port or Linux can0")
    parser.add_argument("--ids", type=lambda v: int(v, 0), nargs="+", default=[0, 1])
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--once", action="store_true",
                        help="Submit one read per ID, without application retries (kernel retries may still occur)")
    parser.add_argument("--no-brs", action="store_true",
                        help="Diagnostic: send CAN FD reads at nominal bitrate; firmware replies remain FD+BRS")
    args = parser.parse_args(argv)
    if (not math.isfinite(args.seconds) or not 0 < args.seconds <= 60
            or any(not 0 <= driver_id <= 255 for driver_id in args.ids)
            or len(set(args.ids)) != len(args.ids)):
        parser.error("Use unique IDs 0..255 and a duration >0 and <=60 seconds")
    tx_mode = "FD, BRS off (1M)" if args.no_brs else "FD+BRS (1M/5M)"
    print(f"TX={tx_mode}. Sending only driver-ID read requests; replies/status use FD+BRS.")
    error_samples = {}

    def remember_error(message):
        # Group counter-byte changes rather than flooding a busy terminal.
        key = (message.arbitration_id, bytes(message.data[:6]))
        if key in error_samples:
            error_samples[key][1] += 1
        elif len(error_samples) < 16:
            error_samples[key] = [describe_error(message), 1]

    try:
        with can.Bus(**connection_options(args.interface, args.channel)) as bus:
            results, errors = probe(bus, args.ids, args.seconds,
                                    brs=not args.no_brs, once=args.once,
                                    on_error=remember_error)
    except (can.CanError, OSError, ValueError, RuntimeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    passed = errors == 0
    for driver_id, result in results.items():
        ok = result["reply"] and result["status_count"] > 0
        passed = passed and ok
        payload = result["last_status"]
        print(f"ID {driver_id}: {'PASS' if ok else 'FAIL'}; "
              f"TX submissions={result['requests_sent']} (not ACK confirmation), "
              f"FD+BRS read reply={result['reply']}, "
              f"8-byte status frames={result['status_count']}, data={payload.hex(' ')}")
        if payload and payload[7]:
            print(f"  Motor reports error 0x{payload[7]:02X}; a transport PASS is not motor-health OK.")
    print(f"Error frames seen: {errors} (backend-dependent; not a bus error-counter check)")
    for description, count in error_samples.values():
        print(f"  {count}x {description}")
    if not passed:
        print("Check FD adapter/firmware, IDs, 1M/5M, BRS, wiring and status broadcast enable.")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
