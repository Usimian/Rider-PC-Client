#!/usr/bin/env python3
"""Spam cal_start + unload commands at /dev/ttyUSB0 to race the ESP32 boot.

Goal: have these commands sitting on the UART when the firmware comes up,
so cal mode gets entered as quickly as possible after servo init, before
servos can stall against mechanical limits.

USAGE:
  1. Robot powered OFF, servos plugged in.
  2. Start this script.
  3. Power on the robot.
  4. Listen. If you hear a sustained loud stall/buzz lasting more
     than ~1 second, KILL POWER IMMEDIATELY (servos may be burning).
  5. If servos go limp quickly, success. Stop this script (Ctrl-C),
     hand-position legs, then run the save_zero step.
"""
import sys
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

def build_frame(addr: int, data: int) -> bytes:
    """XGO write frame: 55 00 LEN 00 ADDR DATA CSUM 00 AA, len = total bytes (9 for 1-byte data)."""
    length = 0x09
    payload = bytes([length, 0x00, addr, data])
    csum = (~sum(payload)) & 0xFF
    return b"\x55\x00" + payload + bytes([csum]) + b"\x00\xAA"

CAL_START = build_frame(0x04, 0x01)   # enter calibration mode
UNLOAD    = build_frame(0x20, 0x01)   # unload all motors

def main():
    print(f"Frames pre-built:")
    print(f"  cal_start: {CAL_START.hex(' ')}")
    print(f"  unload:    {UNLOAD.hex(' ')}")
    print(f"\nOpening {PORT} (DTR/RTS held low to avoid resetting ESP32)...")

    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.dtr = False
    ser.rts = False
    ser.timeout = 0
    ser.write_timeout = 0.5
    ser.open()

    print("Streaming cal_start + unload at ~200 Hz. Power on the robot now.")
    print("Press Ctrl-C to stop once servos go limp.\n")

    count = 0
    last_report = time.monotonic()
    try:
        while True:
            try:
                ser.write(CAL_START)
                ser.write(UNLOAD)
                count += 2
            except (serial.SerialException, OSError):
                # ESP32 might be transitioning at power-on; reopen if needed
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(0.05)
                try:
                    ser.open()
                except Exception:
                    pass
                continue

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(f"  sent {count} frames  ({int(count / (now - last_report))} fps)", flush=True)
                count = 0
                last_report = now

            time.sleep(0.005)  # ~200 Hz spam
    except KeyboardInterrupt:
        print("\nStopped. If servos are released, run save_zero next.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
