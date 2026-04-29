#!/usr/bin/env python3
"""Quick test: can xgolib talk to firmware at all? Try several reads."""
from xgolib import XGO
import time

dog = XGO(port='/dev/ttyUSB0', version='xgorider')
time.sleep(0.5)
print(f"Battery: {dog.rider_read_battery()}")
print(f"Firmware: {dog.rider_read_firmware()}")
print(f"Roll: {dog.rider_read_roll()}")
print(f"Pitch: {dog.rider_read_pitch()}")
print(f"read_motor (15 angles): {dog.read_motor()}")
