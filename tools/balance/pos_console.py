#!/usr/bin/env python3
"""Position console for the Rider balancer.

Live-shows the ACTUAL position vs the TARGET setpoint, and lets you send a new
target (commanded position / exact-distance moves). Reads the firmware telemetry
and sends commands on the same serial link.

Run (robot on):
    /home/marc/.xgo-cal/bin/python pos_console.py            # USB-C (/dev/ttyUSB0)
    /home/marc/.xgo-cal/bin/python pos_console.py --port /dev/ttyAMA0   # on the Pi

Type at the prompt:
    start           START BALANCING (polrun 1 + en 1, no gyro cal) -- hold upright, let go
    stop  (or d)    stop balancing (wheels limp)
    <number>        set position SETPOINT (meters, wheel_x frame)   e.g.  0.30
    r <n>           move setpoint RELATIVE by n meters              e.g.  r 0.1
    here            set setpoint to the CURRENT position (re-anchor)
    poshold 5 | poshd 2 | posmax 3 | gcal | ...   any raw firmware command (passed through)
    q               quit
"""
import sys, time, threading, select, re, argparse
import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/ttyUSB0")
ap.add_argument("--baud", type=int, default=115200)
args = ap.parse_args()

s = serial.Serial(args.port, args.baud, timeout=0.2)
latest = {}
lock = threading.Lock()
running = True


def reader():
    while running:
        try:
            ln = s.readline().decode(errors="replace").strip()
        except Exception:
            break
        if not ln.startswith("th="):
            continue
        d = {k: float(v) for k, v in re.findall(r"(\w+)=(-?[\d.]+)", ln)}
        with lock:
            latest.update(d)


def send(c):
    s.write((c + "\n").encode())
    s.flush()


threading.Thread(target=reader, daemon=True).start()
time.sleep(0.3)
send("get")

print(__doc__)
print("--- live (type a target in meters, or a command; 'q' to quit) ---")
try:
    while running:
        with lock:
            d = dict(latest)
        wx = d.get("wx", float("nan")); tgt = d.get("ptgt", float("nan"))
        th = d.get("th", float("nan")); en = int(d.get("en", 0))
        pr = int(d.get("polrun", 0)); ph = d.get("poshold", float("nan"))
        err = (wx - tgt) if (wx == wx and tgt == tgt) else float("nan")
        sys.stdout.write("\r pos=%+.3f  target=%+.3f  err=%+.3f m | pitch=%+.2f en=%d polrun=%d poshold=%.1f   "
                         % (wx, tgt, err, th, en, pr, ph))
        sys.stdout.flush()
        r, _, _ = select.select([sys.stdin], [], [], 0.25)
        if not r:
            continue
        line = sys.stdin.readline().strip()
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        if line in ("start", "go", "b"):
            send("polrun 1"); time.sleep(0.05); send("en 1")
            print("\n-> BALANCING (no gcal) -- let go now.  ('stop' or 'd' to stop)")
        elif line in ("stop",):
            send("d"); print("\n-> STOPPED (wheels limp)")
        elif line == "here":
            with lock:
                cur = latest.get("wx")
            if cur is not None:
                send("ptgt %.4f" % cur); print("\n-> target = here (%.3f)" % cur)
        elif line.startswith(("r ", "rel ")):
            n = float(line.split()[1])
            with lock:
                cur = latest.get("wx", 0.0)
            send("ptgt %.4f" % (cur + n)); print("\n-> target = %.3f (moved %+.3f)" % (cur + n, n))
        elif re.fullmatch(r"-?\d*\.?\d+", line):
            send("ptgt %s" % line); print("\n-> target = %s" % line)
        else:
            send(line); print("\n-> sent: %s" % line)
finally:
    running = False
    time.sleep(0.3)
    s.close()
print("\nbye.")
