#!/usr/bin/env python3
"""Watch leg angles live via xgolib while firmware is running.
Pick up the robot to trigger non-balance mode and observe the runaway.
Ctrl-C to exit."""
from xgolib import XGO
import time

dog = XGO(port='/dev/ttyUSB0', version='xgorider')
print("Reading motor angles every 200ms. Ctrl-C to exit.\n")
print(f"{'time':>6} {'idx0':>8} {'idx1':>8} {'idx2':>8} {'idx3':>8} {'idx4':>8} {'idx5':>8}  (showing first 6)")
t0 = time.time()
try:
    while True:
        angles = dog.read_motor()
        if angles:
            row = " ".join(f"{a:8.2f}" for a in angles[:6])
            print(f"{time.time()-t0:6.1f} {row}")
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\nDone.")
