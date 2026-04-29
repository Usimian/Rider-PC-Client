#!/usr/bin/env python3
"""Trigger XGO Rider calibration: start (torque off) -> wait -> end (commit)."""
from xgolib import XGO
import sys

label = sys.argv[1] if len(sys.argv) > 1 else "?"

dog = XGO(port='/dev/ttyUSB0', version='xgorider')
print(f"[{label}] Sending CALIBRATION start (torque should drop)...")
dog.rider_calibration('start')
input(f"[{label}] Position shafts now. Press ENTER to commit cal...")
print(f"[{label}] Sending CALIBRATION end (commit)...")
dog.rider_calibration('end')
print(f"[{label}] Done. Power off (switch off) and tell Claude to dump NVS.")
