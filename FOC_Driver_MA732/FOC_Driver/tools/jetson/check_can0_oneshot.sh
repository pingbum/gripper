#!/usr/bin/env bash
# Isolate RX baseline and bounded FD read requests with hardware retries off.
set -euo pipefail
diag_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tools_dir=$(dirname -- "$diag_dir")
python_bin="$tools_dir/.venv/bin/python3"
if [[ ! -x "$python_bin" ]]; then
    printf 'Missing Python environment: %s\n' "$python_bin" >&2
    exit 1
fi
if pgrep -f '(^|[[:space:]/])python[0-9.]*[[:space:]]+([^[:space:]]*/)?app[.]py([[:space:]]|$)' >/dev/null; then
    printf 'Close app.py before running this diagnostic.\n' >&2
    exit 1
fi
report_dir=$(mktemp -d /tmp/ma732-oneshot-XXXXXX)
helper="$report_dir/read_can0_tdc"
cc -O2 -Wall -Wextra -Werror -o "$helper" "$diag_dir/read_can0_tdc.c"
restore_needed=0
cleanup() {
    local result=$?
    trap - EXIT
    if (( restore_needed )); then
        printf 'Restoring normal retransmission and bringing can0 UP (no TX; counters may persist):\n'
        if sudo ip link set can0 down && sudo ip link set can0 type can one-shot off && sudo ip link set can0 up; then
            ip -details -statistics link show can0
        else
            printf 'RESTORE FAILED: inspect can0 before opening app.py.\n' >&2
            result=1
        fi
    fi
    rm -f -- "$helper"
    printf 'Report saved: %s/report.txt\n' "$report_dir"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
exec > >(tee "$report_dir/report.txt") 2>&1
printf 'Report: %s/report.txt\n' "$report_dir"
sudo -v
ip -details -statistics link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.800 sjw 4 dbitrate 5000000 dsample-point 0.800 dsjw 2 fd on berr-reporting on restart-ms 0
if ! sudo ip link set can0 type can one-shot on; then
    printf 'STOP: this driver does not accept one-shot. No requests sent.\n'
    sudo ip link set can0 up
    exit 2
fi
restore_needed=1
printf '0x600\n' | sudo tee /sys/class/net/can0/tdc_offset
sudo ip link set can0 up
hw_state=$(sudo "$helper")
printf '%s\n' "$hw_state"
if [[ "$hw_state" != *'DAR=1,'* || "$hw_state" != *'(TDC=1)'* || "$hw_state" != *'(TDCO=6, TDCF=0)'* ]]; then
    printf 'STOP: hardware DAR/TDC does not match the required diagnostic state. No requests sent.\n'
    exit 2
fi

printf 'RX-only baseline, 2 seconds, no requests:\n'
"$python_bin" -B - "$tools_dir" <<'PY'
from collections import Counter
import sys
import time
sys.path.insert(0, sys.argv[1])
import can
from can_fd_check import connection_options, describe_error
status, errors = Counter(), Counter()
with can.Bus(**connection_options('socketcan', 'can0')) as bus:
    until = time.monotonic() + 2
    while time.monotonic() < until:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.is_error_frame:
            errors[describe_error(msg)] += 1
        elif msg.is_rx and msg.is_fd and msg.bitrate_switch and msg.is_extended_id and msg.arbitration_id <= 255 and len(msg.data) == 8:
            status[msg.arbitration_id] += 1
print('FD+BRS status counts:', dict(status))
print('RX-only error frame count:', sum(errors.values()))
for text, count in errors.most_common(8):
    print(f'  {count}x {text}')
PY
ip -details -statistics link show can0

for mode in off on; do
    sudo ip link set can0 down
    sudo ip link set can0 up
    printf 'BRS %s: down/up performed (counters may persist), at most 3 read submissions, hardware retries OFF.\n' "$mode"
    hw_state=$(sudo "$helper")
    printf '%s\n' "$hw_state"
    if [[ "$hw_state" != *'DAR=1,'* || "$hw_state" != *'(TDC=1)'* || "$hw_state" != *'(TDCO=6, TDCF=0)'* ]]; then
        printf 'STOP: required hardware configuration changed.\n'
        exit 2
    fi
    ip -details -statistics link show can0
    brs_args=()
    if [[ "$mode" == off ]]; then brs_args=(--no-brs); fi
    probe_rc=0
    "$python_bin" -B "$tools_dir/can_fd_check.py" --channel can0 --ids 0 --seconds 1.2 "${brs_args[@]}" || probe_rc=$?
    if (( probe_rc >= 128 )); then exit "$probe_rc"; fi
    printf 'Probe exit: %d\n' "$probe_rc"
    sudo "$helper"
    ip -details -statistics link show can0
done
