#!/usr/bin/env python3
"""Test if Lock=1 is the EEPROM commit trigger.

Sequence: write offset -> lock=1 -> read back. Then power-cycle to verify.
ID 22 only.
"""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

PORT = "/dev/ttyUSB0"; BAUD = 1000000
SID = 22 if len(sys.argv) < 2 else int(sys.argv[1])

REG_OFFSET = 0x1F; REG_TORQUE = 0x28; REG_GOAL = 0x2A; REG_LOCK = 0x37; REG_POS = 0x24

def from_2c(raw, bits=16):
    return raw - (1 << bits) if raw & (1 << (bits - 1)) else raw

def encode_bit11_signmag(val):
    mag = abs(val)
    if mag > 2047: raise ValueError(f"|{val}|>2047")
    return (0x800 | mag) if val < 0 else mag

def main():
    port = PortHandler(PORT); port.openPort(); port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    pos, _, _ = pkt.read2ByteTxRx(port, SID, REG_POS)
    offset, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    pos_s = from_2c(pos)
    print(f"=== BEFORE: encoder={pos_s}  offset=0x{offset:04X} ===")

    # Use a small test value for clarity (123 — distinctive, well below 2047)
    test_value = 123
    enc = encode_bit11_signmag(test_value)
    print(f"\nWriting offset=0x{enc:04X} (={test_value}) and committing with Lock=1")

    # Step 1: ensure unlocked, torque off
    pkt.write1ByteTxRx(port, SID, REG_LOCK, 0)
    pkt.write1ByteTxRx(port, SID, REG_TORQUE, 0)
    time.sleep(0.05)

    # Step 2: write offset
    c, e = pkt.write2ByteTxRx(port, SID, REG_OFFSET, enc)
    print(f"  WRITE offset: comm={c} err={e}")
    time.sleep(0.05)

    # Step 3: Lock=1 (commits to EEPROM but also enables torque)
    c, e = pkt.write1ByteTxRx(port, SID, REG_LOCK, 1)
    print(f"  WRITE lock=1: comm={c} err={e}")
    time.sleep(0.5)

    # Read back
    pos2, _, _ = pkt.read2ByteTxRx(port, SID, REG_POS)
    offset2, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    lock2, _, _ = pkt.read1ByteTxRx(port, SID, REG_LOCK)
    print(f"\n=== AFTER:  encoder={from_2c(pos2)}  offset=0x{offset2:04X}  lock={lock2} ===")

    print(f"\n*** Power-cycle and re-run with no args to read offset ***")
    print(f"If EEPROM committed, BEFORE will show offset=0x{enc:04X}")

    port.closePort()

if __name__ == "__main__":
    main()
