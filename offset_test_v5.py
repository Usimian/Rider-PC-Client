#!/usr/bin/env python3
"""Test SCS-series Lock at register 0x30 (not 0x37) as the EEPROM commit trigger."""
from scservo_sdk import PortHandler, PacketHandler
import time, sys

PORT = "/dev/ttyUSB0"; BAUD = 1000000
SID = 22 if len(sys.argv) < 2 else int(sys.argv[1])

REG_OFFSET = 0x1F
REG_LOCK_SCS = 0x30   # SCS series lock register
REG_LOCK_STS = 0x37   # STS series lock register
REG_POS = 0x24

def main():
    port = PortHandler(PORT); port.openPort(); port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    pos, _, _ = pkt.read2ByteTxRx(port, SID, REG_POS)
    offset, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    lock_30, _, _ = pkt.read1ByteTxRx(port, SID, REG_LOCK_SCS)
    lock_37, _, _ = pkt.read1ByteTxRx(port, SID, REG_LOCK_STS)
    print(f"=== BEFORE ===")
    print(f"  encoder={pos}  offset=0x{offset:04X}  lock@0x30={lock_30}  lock@0x37={lock_37}")

    test_value = 0x0055  # distinctive: 85
    print(f"\nWrite offset=0x{test_value:04X} ({test_value}) then Lock=1 at register 0x30 (SCS lock)")

    pkt.write1ByteTxRx(port, SID, REG_LOCK_SCS, 0)  # unlock SCS-style
    time.sleep(0.05)
    c, e = pkt.write2ByteTxRx(port, SID, REG_OFFSET, test_value)
    print(f"  WRITE offset: comm={c} err={e}")
    time.sleep(0.05)
    c, e = pkt.write1ByteTxRx(port, SID, REG_LOCK_SCS, 1)
    print(f"  WRITE lock@0x30=1: comm={c} err={e}")
    time.sleep(0.5)

    offset2, _, _ = pkt.read2ByteTxRx(port, SID, REG_OFFSET)
    lock2_30, _, _ = pkt.read1ByteTxRx(port, SID, REG_LOCK_SCS)
    lock2_37, _, _ = pkt.read1ByteTxRx(port, SID, REG_LOCK_STS)
    print(f"\n=== AFTER ===")
    print(f"  offset=0x{offset2:04X}  lock@0x30={lock2_30}  lock@0x37={lock2_37}")

    print(f"\n*** Power-cycle and re-run. If BEFORE shows offset=0x{test_value:04X}, this works ***")
    port.closePort()

if __name__ == "__main__":
    main()
