#!/usr/bin/env python3
"""Dump full register space for servo 12, then prompt to move, then dump again
and report which addresses changed.
"""
from scservo_sdk import PortHandler, PacketHandler

PORT = "/dev/ttyUSB0"
BAUD = 1000000
SID = 12

def snap(port, pkt):
    out = {}
    for addr in range(0x00, 0x80):
        v, c, e = pkt.read1ByteTxRx(port, SID, addr)
        out[addr] = v
    return out

def main():
    port = PortHandler(PORT)
    port.openPort()
    port.setBaudRate(BAUD)
    pkt = PacketHandler(0)

    input("Position leg 12 at POSITION A. Press Enter to snapshot.")
    a = snap(port, pkt)
    print(f"  Got {sum(1 for v in a.values() if v != 0)} non-zero bytes")

    input("Position leg 12 at POSITION B (very different). Press Enter to snapshot.")
    b = snap(port, pkt)

    print("\nAddresses that CHANGED between A and B:")
    any_change = False
    for addr in range(0x00, 0x80):
        if a[addr] != b[addr]:
            print(f"  0x{addr:02X}: {a[addr]:3d} -> {b[addr]:3d}")
            any_change = True
    if not any_change:
        print("  (no bytes changed — encoder isn't reporting)")

    port.closePort()

if __name__ == "__main__":
    main()
