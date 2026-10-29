#!/usr/bin/env python3
"""Run the firmware's wheel FREQUENCY SWEEP on the floor and save its capture (fw 2026-10-19, main.cpp 'chirp').

While the robot balances in place, the firmware adds a known log-swept sine (default +/-60 raw cmd, 1 -> 10 Hz
over 20 s, faded in/out) to the forward wheel command and records every 2nd cycle: the injected value, both
wheels' final commands and reported speeds, the encoder position, pitch and pitch rate. Because the injection is
known, the wheel's true response can be separated from the controller's (sim/../ analysis: H = S_yr / S_ur).

The robot must already be BALANCING (the firmware refuses otherwise). Hands off the stick during the sweep.
--on-drive: wait until the robot drives steadily at a PARTIAL stick (0.08-0.25 m/s), then sweep -- the wheels
then stay clear of their dead zone, separating the wheel's own response from its behaviour near standstill.
--full (with --on-drive): wait for FULL stick (>= 0.28 m/s) instead -- easy to hold steady against the end-stop.
Run (workstation):  /usr/bin/python3 tools/capture/chirp_capture.py out.csv [amp f0 f1 secs] [--on-drive --dir fwd|rev
                    [--full]]
After saving, it prints the robot's speed through the sweep and says DRIVE NOT HELD if a driving sweep lost its drive.
The broker comes from gui/rider_config.ini, as for tools/diag/mqtt_monitor.py.
"""
import sys, os, time, json, configparser
import paho.mqtt.client as mqtt

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
ON_DRIVE = "--on-drive" in sys.argv   # wait for a steady partial-stick drive, then sweep (wheels clear of the dead zone)
FULL = "--full" in sys.argv           # with --on-drive: wait for FULL stick instead (easier to hold steady)
# --dir fwd|rev (with --on-drive, required): only a drive that way starts the sweep (2026-10-29: a direction-blind
# trigger let a forward line-up drive start the backward sweep, which then ran mostly after the drive)
DIR = {"fwd": 1.0, "rev": -1.0}[sys.argv[sys.argv.index("--dir") + 1]] if "--dir" in sys.argv else None
if ON_DRIVE and DIR is None:
    sys.exit("--on-drive needs --dir fwd or --dir rev")
_args = [a for a in sys.argv[1:] if not a.startswith("--") and a not in ("fwd", "rev")]
OUT = _args[0] if _args else "chirp.csv"
AMP, F0, F1, SECS = ([float(x) for x in _args[1:5]] + [60.0, 1.0, 10.0, 20.0][len(_args[1:5]):])
COLS = ["i", "t_us", "inj", "cmdL", "cmdR", "vL", "vR", "wx_01mm", "th_x100", "rate_x10"]
st = {"en": None, "ctrl": "?", "rows": [], "done": False, "header": False, "vdes": 0.0, "drive_since": None}


def line(c, s):
    c.publish("rider/control/line", json.dumps({"line": s}), qos=1)


def on_connect(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/telem"); c.subscribe("rider/debug/dcap")


def on_message(c, u, m):
    pl = m.payload.decode(errors="replace").strip()
    if m.topic == "rider/debug/telem" and pl.startswith("th="):
        d = dict(x.split("=", 1) for x in pl.split() if "=" in x)
        st["en"] = d.get("en"); st["ctrl"] = d.get("ctrl", "?")
        try:
            st["vdes"] = float(d.get("vdes", 0))
        except ValueError:
            st["vdes"] = 0.0
        v = st["vdes"] * (DIR or 1.0)                     # + = the asked-for direction
        steady = (v >= 0.28) if FULL else (0.08 <= v <= 0.25)   # full / partial stick, that way
        st["drive_since"] = (st["drive_since"] or time.time()) if steady else None
    elif pl.startswith("# dcapc done"):
        st["done"] = True
    elif pl.startswith("# dcapc n="):
        st["header"] = True
    elif pl.startswith("# dcapc REFUSED"):
        print("firmware:", pl, flush=True)
    elif pl.startswith("# dcapc "):
        p = pl.split()
        if len(p) == 2 + len(COLS):
            st["rows"].append(",".join(p[2:]))


cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="chirp_capture")
cl.on_connect = on_connect; cl.on_message = on_message
cl.connect(BROKER, 1883, 30); cl.loop_start()
t0 = time.time()
while st["en"] is None and time.time() - t0 < 5:
    time.sleep(0.1)
if st["en"] != "1" and not ON_DRIVE:          # --on-drive waits below for balancing AND the drive
    sys.exit("robot is not balancing (en=%s) -- start balancing first" % st["en"])
if ON_DRIVE:
    print("waiting for a steady %s drive (%s for 1.5 s) ..." % (("FULL-stick", ">= 0.28 m/s") if FULL else
                                                               ("PARTIAL-stick", "0.08-0.25 m/s")), flush=True)
    while not (st["en"] == "1" and st["drive_since"] and time.time() - st["drive_since"] > 1.5):
        time.sleep(0.05)
    print("driving at %.2f m/s -- sweep starts now, KEEP the stick steady" % st["vdes"], flush=True)
print("balancing in %s; sweep +/-%.0f cmd, %.1f -> %.1f Hz over %.0f s -- %s"
      % (st["ctrl"], AMP, F0, F1, SECS, "keep the stick steady" if ON_DRIVE else "hands off the stick"), flush=True)
for k, v in (("chirpa", AMP), ("chirpf0", F0), ("chirpf1", F1), ("chirpt", SECS)):
    line(cl, "%s %g" % (k, v))
time.sleep(0.3)
line(cl, "chirp 1")
time.sleep(SECS + 1.0)
print("sweep done -> fetching the capture", flush=True)
line(cl, "chirpdump")
t0 = time.time()
while not st["done"] and time.time() - t0 < 30:
    time.sleep(0.2)
cl.loop_stop()
with open(OUT, "w") as f:
    f.write("ctrl," + ",".join(COLS) + "\n")
    for r in st["rows"]:
        f.write("%s,%s\n" % (st["ctrl"], r))
print("%d samples -> %s%s" % (len(st["rows"]), OUT, "" if st["done"] else "  (INCOMPLETE: no 'done' line)"), flush=True)
# the recording's own check: the robot's speed through the sweep (both wheels' registers), per 2 s. A driving sweep
# must hold its drive the whole way, or the wheels' response mixes with a stop.
if st["rows"]:
    a = [[float(x) for x in r.split(",")] for r in st["rows"]]
    t = [(r[COLS.index("t_us")] - a[0][COLS.index("t_us")]) * 1e-6 for r in a]
    v = [(r[COLS.index("vL")] - r[COLS.index("vR")]) / 2 / 327.6 for r in a]
    parts = []
    for k in range(0, int(t[-1]) + 1, 2):
        vs = [vv for tt, vv in zip(t, v) if k <= tt < k + 2]
        if vs:
            parts.append(sum(vs) / len(vs))
    print("speed through the sweep, per 2 s: " + "  ".join("%+.2f" % p for p in parts) + " m/s", flush=True)
    if ON_DRIVE and min(p * DIR for p in parts) < 0.2:
        print("DRIVE NOT HELD: the robot slowed below 0.2 m/s during the sweep -- this recording is not a driving "
              "sweep; record it again", flush=True)
