#!/usr/bin/env python3
"""Clean offset-write test with detached servos.

ID 22 only.
Procedure:
  1. Read encoder + offset + goal
  2. Disable torque, ensure lock=0
  3. Write Homing_Offset = encoder_position (bit-11 sign-mag)
  4. Wait, read back
  5. Print verification — power-cycle, re-run this script to see if offset persists
"""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

PORT = "/dev/ttyUSB0"; BAUD = 1000000
SID = 22 if len(sys.argv) < 2 else int(sys.argv[1])

REG_OFFSET = 0x1F
REG_TORQUE = 0x28
REG_GOAL   = 0x2A
REG_LOCK   = 0x37
REG_POS    = 0x24

def from_2c(raw, bits=16):
    return raw - (1 << bits) if raw & (1 << (bits - 1)) else raw

def encode_bit11_signmag(val):
    mag = abs(val)
    if mag > 2047:
        raise ValueError(f"Magnitude {mag} > 2047")
    return (0x800 | mag) if val < 0 else mag

def main():
    port = PortHandler(PORT); port.openPort(); port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    print(f"=== ID {SID} BEFORE ===")
    pos, _, _ = pkt.read2ByteTxRx(port, SID, REG_POS)
    offset, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    goal, _, _ = pkt.read2ByteTxRx(port, SID, REG_GOAL)
    torque, _, _ = pkt.read1ByteTxRx(port, SID, REG_TORQUE)
    lock, _, _ = pkt.read1ByteTxRx(port, SID, REG_LOCK)
    pos_s = from_2c(pos)
    print(f"  encoder@0x24 = {pos_s:5d}   offset@0x1F = 0x{offset:04X}   goal@0x2A = 0x{goal:04X}")
    print(f"  torque = {torque}   lock = {lock}")

    print("\nDisable torque, lock=0 (safety)")
    pkt.write1ByteTxRx(port, SID, REG_TORQUE, 0)
    pkt.write1ByteTxRx(port, SID, REG_LOCK, 0)
    time.sleep(0.05)

    new_offset = encode_bit11_signmag(pos_s)
    print(f"\nWrite Homing_Offset = {pos_s} (encoded 0x{new_offset:04X}) to register 0x1F")
    print("(servo may spin freely — leg is detached, so this is fine)")
    c, e = pkt.write2ByteTxRx(port, SID, REG_OFFSET, new_offset)
    print(f"  comm={c} err={e}")
    time.sleep(0.5)

    print(f"\n=== ID {SID} AFTER ===")
    pos2, _, _ = pkt.read2ByteTxRx(port, SID, REG_POS)
    offset2, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    pos2_s = from_2c(pos2)
    print(f"  encoder@0x24 = {pos2_s:5d}   offset@0x1F = 0x{offset2:04X}")

    print("\n=== Persistence test ===")
    print("1. Power-cycle the robot")
    print("2. Re-run this script")
    print(f"3. If 'offset' in BEFORE = 0x{new_offset:04X}, EEPROM persisted")
    print(f"   If 'offset' in BEFORE = 0x0000 (factory) or original, EEPROM didn't commit")

    port.closePort()

if __name__ == "__main__":
    main()
