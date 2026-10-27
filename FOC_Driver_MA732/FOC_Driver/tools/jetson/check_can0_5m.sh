#!/usr/bin/env bash
# One controlled CAN0 recovery + 5 Mbps BRS-on driver-ID read. No motor writes.
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
if [[ ! -r /sys/class/net/can0/tdc_offset ]]; then
    printf 'Expected Jetson can0/tdc_offset node is missing.\n' >&2
    exit 1
fi

report_dir=$(mktemp -d /tmp/ma732-can5m-XXXXXX)
helper="$report_dir/read_can0_tdc"
trap 'rm -f -- "$helper"' EXIT
cc -O2 -Wall -Wextra -Werror -o "$helper" "$diag_dir/read_can0_tdc.c"
exec > >(tee "$report_dir/report.txt") 2>&1
printf 'Report: %s/report.txt\n' "$report_dir"
printf 'Before recovery:\n'
ip -details -statistics link show can0
cat /sys/class/net/can0/tdc_offset

sudo -v
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.800 sjw 4 dbitrate 5000000 dsample-point 0.800 dsjw 2 fd on berr-reporting on restart-ms 0
printf '0x600\n' | sudo tee /sys/class/net/can0/tdc_offset
sudo ip link set can0 up
printf 'After recovery, before any test TX:\n'
cat /sys/class/net/can0/tdc_offset
hw_state=$(sudo "$helper")
printf '%s\n' "$hw_state"
ip -details -statistics link show can0
if [[ "$hw_state" != *'(TDC=1)'* || "$hw_state" != *'(TDCO=6, TDCF=0)'* ]]; then
    printf 'STOP: hardware TDC does not match 0x600. No read request was sent.\n'
    printf 'Boot command line: '
    cat /proc/cmdline
    printf 'Report saved: %s/report.txt\n' "$report_dir"
    exit 2
fi

printf 'Sending one FD+BRS driver-ID read to ID 0:\n'
probe_rc=0
"$python_bin" -B "$tools_dir/can_fd_check.py" --interface socketcan --channel can0 --ids 0 --once --seconds 2 || probe_rc=$?
printf 'After test (probe exit=%d):\n' "$probe_rc"
cat /sys/class/net/can0/tdc_offset
sudo "$helper"
ip -details -statistics link show can0
printf 'Report saved: %s/report.txt\n' "$report_dir"
exit "$probe_rc"
