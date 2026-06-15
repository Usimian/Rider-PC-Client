#!/usr/bin/env python3
"""Pi-side control of the Rider self-balance firmware (esp32_rider_fw).

Runs on the Raspberry Pi and talks to the ESP32 using the firmware's
newline-terminated ASCII protocol. The Pi is wired to the ESP32's UART1
(ESP32 IO4=RX / IO5=TX) and sees it as /dev/ttyAMA0 (a.k.a. /dev/serial0).
The firmware mirrors the same command/telemetry interface on UART1 (Pi) AND
UART0 (USB-C, /dev/ttyUSB0) — so only ONE host should send at a time (stop the
stock rider-controller service / don't drive from the workstation at once).

Commands the firmware understands (one per line):
  en 1 | en 0   enable/disable balance       d        quick disable
  kp <v>        proportional gain            kd <v>   derivative gain
  umax <v>      torque clamp                 set <deg> tilt setpoint
  cap           setpoint = current tilt      pol <v>  polarity (sign of v)
  get           report state
Telemetry streams continuously as:  th=.. rate=.. u=.. en=.. Kp=.. Kd=.. set=.. pol=.. Umax=..

Examples:
  python3 pi_balance_ctl.py kp 18 ; python3 pi_balance_ctl.py set -1.6
  python3 pi_balance_ctl.py en 1               # enable
  python3 pi_balance_ctl.py watch              # stream telemetry
"""
import serial, sys, time

PORT = "/dev/ttyAMA0"     # adjust if your Pi exposes it elsewhere (/dev/serial0)
BAUD = 115200


class RiderBalance:
    def __init__(self, port=PORT, baud=BAUD):
        self.s = serial.Serial(port, baud, timeout=0.2)

    def send(self, line):
        self.s.write((line.strip() + "\n").encode())

    # convenience setters
    def kp(self, v):        self.send(f"kp {v}")
    def kd(self, v):        self.send(f"kd {v}")
    def umax(self, v):      self.send(f"umax {v}")
    def setpoint(self, v):  self.send(f"set {v}")
    def capture(self):      self.send("cap")
    def polarity(self, v):  self.send(f"pol {v}")
    def enable(self):       self.send("en 1")
    def disable(self):      self.send("en 0")
    def get(self):          self.send("get")

    def telemetry(self):
        """Return latest telemetry as a dict, or None."""
        ln = self.s.readline().decode(errors="replace").strip()
        if not ln.startswith("th="):
            return None
        d = {}
        for tok in ln.replace("|", " ").split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                try: d[k] = float(v)
                except ValueError: d[k] = v
        return d


def main():
    rb = RiderBalance()
    args = sys.argv[1:]
    if not args:
        print(__doc__); return
    if args[0] == "watch":
        try:
            while True:
                t = rb.telemetry()
                if t: print(t)
        except KeyboardInterrupt:
            print()
        return
    # pass through any raw command line, then echo the firmware's ack
    rb.send(" ".join(args))
    time.sleep(0.15)
    while rb.s.in_waiting:
        ln = rb.s.readline().decode(errors="replace").strip()
        if ln.startswith("#"):
            print(ln)


if __name__ == "__main__":
    main()
