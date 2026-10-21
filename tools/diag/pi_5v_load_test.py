#!/usr/bin/env python3
"""5 V at the Pi (CM5) under load: 10 s idle, 60 s with all 4 cores busy, 10 s idle, sampling the CM5's
own EXT5V sensor, the arm clock and the throttle flags every ~0.5 s. Prints per phase the median / min
voltage, how many samples were in under-voltage / throttled, and the kernel alarms during the test.

Used 2026-10-19 (the 2 GHz cap) and 2026-10-21 (a bad 5 V lead contact: min 4.60 V and under-voltage
half the time before reseating the connectors, 4.72 V and none after) -- see docs/HARDWARE.md -> Power.
Run with the robot DISABLED (balancing off):   tools/diag/pi_5v_load_test.py [host]   (default: rider)
"""
import statistics as st
import subprocess
import sys

# Runs on the Pi. Phases are timed on /proc/uptime (monotonic): the Pi's wall clock jumps when it first
# syncs after boot, which cut an early test short.
SAMPLER = r"""
up() { cut -d' ' -f1 /proc/uptime | cut -d. -f1; }
sample() {
  local end=$(( $(up) + $2 ))
  while [ "$(up)" -lt "$end" ]; do
    v=$(vcgencmd pmic_read_adc EXT5V_V | grep -o '=[0-9.]*' | tr -d =)
    f=$(vcgencmd measure_clock arm | cut -d= -f2)
    th=$(vcgencmd get_throttled | cut -d= -f2)
    echo "S $1 $v $f $th"
    sleep 0.4
  done
}
uv0=$(journalctl -k --no-pager | grep -c 'Undervoltage detected')
sample idle 10
for i in 1 2 3 4; do timeout 60 sh -c 'while :; do :; done' & done
sample load 60
wait
sample after 10
echo "ALARMS $(( $(journalctl -k --no-pager | grep -c 'Undervoltage detected') - uv0 ))"
vcgencmd measure_temp
"""


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "rider"
    out = subprocess.run(["ssh", host, "bash -s"], input=SAMPLER, capture_output=True, text=True,
                         check=True).stdout
    rows = {}
    for line in out.splitlines():
        p = line.split()
        if p and p[0] == "S" and len(p) == 5:
            rows.setdefault(p[1], []).append((float(p[2]), int(p[3]), int(p[4], 16)))
        elif line.strip():
            print(line)
    for ph in ("idle", "load", "after"):
        r = rows.get(ph, [])
        if not r:
            continue
        v = [x[0] for x in r]
        print(f"{ph:5s} n={len(r):3d}  5V median {st.median(v):.3f}  min {min(v):.3f}  max {max(v):.3f}  "
              f"clock {st.mean(x[1] for x in r) / 1e9:.2f} GHz  under-volt {sum(x[2] & 1 for x in r)}  "
              f"throttled {sum((x[2] >> 2) & 1 for x in r)}")


if __name__ == "__main__":
    main()
