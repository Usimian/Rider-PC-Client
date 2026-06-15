#!/usr/bin/env python3
"""Exit calibration mode and save offsets.

Use this AFTER cal_spam.py succeeded and you've hand-positioned the
legs to the factory neutral pose:
  - body flush with floor
  - thighs at 90° to body
  - calves at 90° to thighs
  - calves on ground

Sends register 0x04 = 0x00 (exit cal mode -> firmware records the
current physical positions as the new zero offsets).
"""
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

def build_frame(addr: int, data: int) -> bytes:
    length = 0x09
    payload = bytes([length, 0x00, addr, data])
    csum = (~sum(payload)) & 0xFF
    return b"\x55\x00" + payload + bytes([csum]) + b"\x00\xAA"

def main():
    cal_end = build_frame(0x04, 0x00)
    print(f"Sending cal_end: {cal_end.hex(' ')}")

    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.dtr = False
    ser.rts = False
    ser.timeout = 0.2
    ser.open()

    # Send a few times to be robust
    for i in range(5):
        ser.write(cal_end)
        time.sleep(0.05)

    ser.close()
    print("Sent. Power-cycle the robot and verify it stands correctly.")

if __name__ == "__main__":
    main()
