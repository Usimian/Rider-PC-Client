#!/usr/bin/env python3
"""Capture the wheel step response from the ESP32 ('stepcap' firmware) and
extract the sim-to-real actuator params: latency, first-order lag (tau), and
whether the command acts as torque (velocity ramps) or velocity (settles).

Run on the workstation over USB-C (robot ON, on the stand, wheels free):
    /home/marc/.xgo-cal/bin/python measure_actuator.py [step_cmd]

The wheels spin briefly during the capture -- keep them free.
"""
import sys
import time
import serial

PORT = "/dev/ttyUSB0"
step = int(sys.argv[1]) if len(sys.argv) > 1 else 200

s = serial.Serial(PORT, 115200, timeout=2.0)
time.sleep(0.4)
s.reset_input_buffer()
s.write(b"lock 11\n")            # poll only the left wheel -> full-rate sampling of one wheel
time.sleep(0.2)
s.reset_input_buffer()
s.write(f"stepcap {step}\n".encode())
s.flush()

rows = []
t0 = time.time()
while time.time() - t0 < 8.0:
    line = s.readline().decode(errors="replace").strip()
    if line.startswith("# cap done"):
        break
    if line.startswith("# cap "):
        parts = line.split()
        if len(parts) == 6:          # "# cap i t_us cmd vel"
            try:
                rows.append((int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])))
            except ValueError:
                pass
s.write(b"lock 0\n")               # restore alternating poll
s.close()

if not rows:
    print("NO CAPTURE DATA -- is the stepcap firmware flashed and robot powered?")
    sys.exit(1)

rows.sort(key=lambda r: r[0])        # by sample index
idx = [r[0] for r in rows]
t_us = [r[1] for r in rows]
cmd = [r[2] for r in rows]
vel = [r[3] for r in rows]

# step happens at the first sample whose cmd != 0
step_i = next((k for k, c in enumerate(cmd) if c != 0), None)
print(f"captured {len(rows)} samples, mean period "
      f"{(t_us[-1]-t_us[0])/(len(rows)-1)/1000:.2f} ms, step cmd={step} at sample {step_i}\n")

print(" i   t_ms   cmd    vel")
for k in range(len(rows)):
    mark = " <- step" if k == step_i else ""
    print(f"{idx[k]:3d} {t_us[k]/1000:6.1f} {cmd[k]:5d} {vel[k]:6d}{mark}")

if step_i is not None:
    t_step = t_us[step_i]
    base = vel[:step_i]
    noise = max(3, int(2 * (max(map(abs, base)) if base else 0) + 3))
    resp_i = next((k for k in range(step_i, len(rows)) if abs(vel[k]) > noise), None)
    final = sum(vel[-10:]) / 10.0
    print("\n--- estimate ---")
    if resp_i is not None:
        print(f"latency      ~ {(t_us[resp_i]-t_step)/1000:.1f} ms "
              f"(vel left baseline at sample {idx[resp_i]})")
    if final:
        target = 0.632 * final
        tau_i = next((k for k in range(step_i, len(rows)) if abs(vel[k]) >= abs(target)), None)
        if tau_i is not None:
            print(f"tau (63%)    ~ {(t_us[tau_i]-t_step)/1000:.1f} ms")
    print(f"final vel    ~ {final:.0f} raw   (last-10 mean)")
    # torque vs velocity: settled (flat tail) => velocity-like; still rising => torque-like
    tail = vel[-20:]
    rising = abs(tail[-1]) - abs(tail[0])
    span = abs(final) if final else 1
    print(f"tail trend   : {rising:+.0f} raw over last 20 samples "
          f"({'still RAMPING -> torque-like' if abs(rising) > 0.15*span else 'SETTLED -> velocity-like'})")
