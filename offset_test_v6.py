#!/usr/bin/env python3
"""Test REG_WRITE + ACTION pattern as the EEPROM commit on these servos."""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

PORT = "/dev/ttyUSB0"; BAUD = 1000000
SID = 22 if len(sys.argv) < 2 else int(sys.argv[1])
REG_OFFSET = 0x1F

def main():
    port = PortHandler(PORT); port.openPort(); port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    offset, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    pos, _, _ = pkt.read2ByteTxRx(port, SID, 0x24)
    print(f"BEFORE: offset=0x{offset:04X}  encoder={pos if pos<0x8000 else pos-0x10000}")

    test_value = 0x002A  # 42 — distinctive
    data = [test_value & 0xFF, (test_value >> 8) & 0xFF]
    print(f"\nREG_WRITE 0x{test_value:04X} ({test_value}) to 0x1F (deferred)...")
    c, e = pkt.regWriteTxRx(port, SID, REG_OFFSET, 2, data)
    print(f"  REG_WRITE: comm={c} err={e}")
    time.sleep(0.05)

    print(f"\nACTION (commit pending REG_WRITE)...")
    result = pkt.action(port, SID)
    print(f"  ACTION: {result}")
    time.sleep(0.5)

    after, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    print(f"\nAFTER:  offset=0x{after:04X}")

    print(f"\nPower-cycle and re-run. If BEFORE shows 0x{test_value:04X}, REG_WRITE+ACTION commits.")
    port.closePort()

if __name__ == "__main__":
    main()
