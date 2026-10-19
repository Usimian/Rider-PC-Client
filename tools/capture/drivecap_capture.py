#!/usr/bin/env python3
"""Auto-capture the firmware's high-rate drive buffer (commanded vs actual wheel
velocity + pitch) over the Pi MQTT broker -- works while the robot drives
UNTETHERED (USB-C unplugged). No timing coordination needed from the driver.

Subscribes to the Pi broker: rider/debug/telem (to detect a sustained drive) and
rider/debug/dcap (the dump). On a sustained drive it publishes `drivecap` to
rider/control/line, collects the `# dcap` block, and appends it to CSV tagged
fwd/rev. Just drive forward a few times, then backward.

--spin (2026-10-19): trigger on a SPIN in place instead (a yaw-rate command, no drive) -- for the
turning shudder. The 'dir' column then reads spin-<POL|LQR>-g<turn output>: the captured cmd is the
balance output u; the left wheel's actual command is u plus that (near-constant) turn output.

Run (workstation):  /usr/bin/python3 tools/capture/drivecap_capture.py [out.csv] [--spin]
The broker comes from gui/rider_config.ini, as for tools/diag/mqtt_monitor.py.
"""
import sys, os, time, json, configparser
import paho.mqtt.client as mqtt
from collections import deque

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
SPIN = "--spin" in sys.argv
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
OUT = _args[0] if _args else "drivecap.csv"
THR = 0.12
f = open(OUT, "w", buffering=1)
# the firmware's '# dcap' fields, in order (main.cpp logdump; 'cmd' = the balance output u)
COLS = ["i", "t_us", "cmd", "wid", "v", "pitch_cdeg", "rate_x10", "cmdL", "cmdR", "vR",
        "pol_pitch_x100", "pol_rate_x10", "pol_xvel_mm", "pol_int_m", "pol_bias_x100"]
f.write("episode,dir," + ",".join(COLS) + "\n")
st = {"ep": 0, "last": 0.0, "cap": False, "dir": "", "rows": [], "seen": False,
      "armed": 0.0, "dumped": True}
hist = deque(maxlen=6)


def on_connect(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/telem"); c.subscribe("rider/debug/dcap")
    print("monitoring Pi broker for driving (Ctrl-C to stop) ...", flush=True)


def on_message(c, u, m):
    pl = m.payload.decode(errors="replace").strip()
    if m.topic == "rider/debug/dcap":
        if pl.startswith("# dcap done"):
            if st["cap"]:
                for r in st["rows"]:
                    f.write("%d,%s,%s\n" % (st["ep"], st["dir"], r))
                print("   episode %d (%s): %d samples saved" % (st["ep"], st["dir"], len(st["rows"])), flush=True)
                st["ep"] += 1; st["cap"] = False; st["rows"] = []; st["last"] = time.time(); hist.clear()
        elif pl.startswith("# dcap ") and st["cap"]:
            p = pl.split()
            if len(p) == 2 + len(COLS):      # '# dcap' + the fields (fw 2026-10-19 added cmdL..pol_bias_x100)
                st["rows"].append(",".join(p[2:]))
        return
    if not pl.startswith("th="):
        return
    d = dict(x.split("=") for x in pl.split() if "=" in x)
    try:
        vdes = float(d.get("vdes", 0)); en = int(float(d.get("en", 0)))
        yrc = float(d.get("yrc", 0)); gturn = float(d.get("gturn", 0))
    except ValueError:
        return
    if not st["seen"]:
        st["seen"] = True
        print("  telemetry flowing (en=%d vdes=%.2f) -- ready to %s" % (en, vdes, "spin" if SPIN else "drive"),
              flush=True)
    # spin mode: the 'signal' is the yaw-rate command while not driving (0 otherwise)
    hist.append((yrc if abs(vdes) < 0.05 else 0.0) if SPIN else vdes)
    # The fw HOLDS the drivecap buffer until an explicit 'logdump' (which streams the
    # # dcap block out Serial1 = Pi UART -> bridge -> rider/debug/dcap). The old tool never
    # sent it, so nothing ever came back. Send logdump ~4s after arming (buffer is 600
    # samples @ ~163Hz = 3.7s), then collect the dump.
    if st["cap"] and not st["dumped"] and time.time() - st["armed"] > 4.0:
        st["dumped"] = True
        print("   -> logdump (fetching buffer)", flush=True)
        c.publish("rider/control/line", json.dumps({"line": "logdump"}), qos=1)
    if (en and not st["cap"] and len(hist) == hist.maxlen
            and all(abs(x) > THR for x in hist)
            and all((x > 0) == (hist[0] > 0) for x in hist)
            and time.time() - st["last"] > 2.5):
        if SPIN:
            st["dir"] = "spin-%s-g%+.0f" % (d.get("ctrl", "?"), gturn)
        else:
            st["dir"] = "fwd" if hist[0] > 0 else "rev"
        st["cap"] = True; st["rows"] = []
        st["armed"] = time.time(); st["dumped"] = False
        print("  %s detected -> drivecap (keep going ~4s)" % st["dir"], flush=True)
        c.publish("rider/control/line", json.dumps({"line": "drivecap"}), qos=1)


cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="drivecap")
cl.on_connect = on_connect
cl.on_message = on_message
cl.connect(BROKER, 1883, 30)
cl.loop_forever()
