#!/usr/bin/env python3
"""XGO Rider leg calibration over USB-C UART (CH340 → STM32).

Run with the PC venv:
  ~/.xgo-cal/bin/python calibrate_legs_pc.py

Preconditions:
  - rider-controller.service is stopped on the Pi (so it doesn't fight the bus)
  - Robot body fully supported in air, legs swinging free
  - Servos plugged in
"""
import sys
import time
from xgolib import XGO

BAR = "=" * 60

def banner(msg):
    print()
    print(BAR)
    print(f"  {msg}")
    print(BAR)
    print()

def main():
    print("Connecting to XGO Rider on /dev/ttyUSB0 ...")
    dog = XGO(port="/dev/ttyUSB0", version="xgorider")
    fw = dog.read_firmware()
    print(f"Firmware: {fw}")
    if not fw or fw[0] != 'R':
        print("ERROR: not a Rider (or no response). Aborting.")
        sys.exit(2)

    print("unload_allmotor()")
    dog.unload_allmotor()

    print("rider_calibration('start')")
    dog.rider_calibration('start')

    banner(">>> CALIBRATION STARTED — POSITION LEGS NOW (10s) <<<")
    time.sleep(10)

    dog.rider_calibration('end')
    banner(">>> CALIBRATION SAVED — DONE <<<")

if __name__ == "__main__":
    main()
