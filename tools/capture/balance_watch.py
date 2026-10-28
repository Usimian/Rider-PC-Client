#!/usr/bin/env python3
"""Watch a balancing test live at the full loop rate: record back-to-back windows AND analyse each one as it
arrives (2026-10-28, the first LQR2 floor runs).

The 10 Hz telemetry is too slow to show a fast oscillation (it aliases it into noise). This arms the firmware's
drive capture ('drivecap': 600 samples, every control cycle, ~3 s), fetches it ('logdump'), saves it, and prints
one line per window:

  ctrl/en       the controller and whether it balanced in this window
  pitch         mean and sd (deg, the firmware's fused pitch)
  rate          the gyro's rms (deg/s) and its strongest frequency in 1.5-95 Hz with how far it stands out
                (peak / band mean); a sharp peak = an oscillation, ~1-3 = broadband noise
  cmd           the balance output: mean, sd, strongest frequency, and sign flips per second (a friction
                compensation chattering around zero flips it tens of times a second)
  speed         the robot's speed (both wheels' registers averaged; turns cancel): sd and mean (mm/s)

Files saved before 2026-10-28 ~11:10 kept t_us to 6 digits only (1-10 ms steps): sample_times() rebuilds them.

Run (workstation):  /usr/bin/python3 tools/capture/balance_watch.py out.csv [windows]   (default 40)
       analyse a saved file:  /usr/bin/python3 tools/capture/balance_watch.py --analyse file.csv
The broker comes from gui/rider_config.ini, as for tools/diag/mqtt_monitor.py.
"""
import sys, os, time, json, configparser
import numpy as np
import paho.mqtt.client as mqtt

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
# the firmware's '# dcap' fields, in order (main.cpp logdump), as in drivecap_capture.py
COLS = ["i", "t_us", "cmd", "wid", "v", "pitch_cdeg", "rate_x10", "cmdL", "cmdR", "vR",
        "pol_pitch_x100", "pol_rate_x10", "pol_xvel_mm", "pol_int_m", "pol_bias_x100", "pacc_x100", "yaw_x10",
        "yrc_x1000", "stick_mms", "plan_acc_x1000", "plan_lean_x100"]
V_PER_MPS = 327.6   # speed register counts per m/s (tools/balance/encoder_axle_check.py)


def peak(x, fs, lo, hi):
    """Strongest frequency of x in [lo, hi] Hz and how far it stands out (peak / band mean)."""
    x = np.asarray(x, float)
    if len(x) < 64 or np.allclose(x, x[0]):
        return float("nan"), float("nan")
    X = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), 1 / fs)
    m = (fr >= lo) & (fr <= min(hi, fs / 2 * 0.95))
    if not m.any():
        return float("nan"), float("nan")
    k = int(np.argmax(X[m]))
    return float(fr[m][k]), float(X[m][k] / (X[m].mean() + 1e-12))


def sample_times(i, t_us):
    """Seconds for one window's samples. The samples are consecutive control cycles (i), so a straight line
    through the logged times recovers each one to ~0.3 % even where t_us was saved rounded (the 6-digit files)."""
    k, c = np.polyfit(i, t_us, 1)
    return (k * (i - i[0])) * 1e-6


def analyse(rows, ctrl="?"):
    """rows: dict of column arrays for one window -> the summary line."""
    if len(rows["t_us"]) < 300 or np.any(np.diff(rows["i"]) != 1):
        return f"{ctrl:4s} incomplete window ({len(rows['t_us'])} samples) -- skipped"
    t = sample_times(rows["i"], rows["t_us"])
    fs = 1.0 / np.median(np.diff(t))
    u = rows["cmd"]
    on = np.mean(u != 0) > 0.5
    pitch = rows["pitch_cdeg"] / 100.0
    rate = rows["rate_x10"] / 10.0
    v = (rows["v"] - rows["vR"]) / 2.0 / V_PER_MPS * 1000      # the right wheel's register is mirrored
    fr, pr = peak(rate, fs, 1.5, 95)
    fu, pu = peak(u, fs, 1.5, 95)
    s = np.sign(u[u != 0])
    flips = np.sum(s[1:] != s[:-1]) / (t[-1] - t[0]) if len(s) > 1 else 0.0
    return (f"{ctrl:4s} {'ON ' if on else 'off'} {fs:3.0f} Hz | pitch {pitch.mean():6.2f} sd {pitch.std():5.2f} | "
            f"rate rms {np.sqrt(np.mean(rate ** 2)):5.1f} peak {fr:5.1f} Hz x{pr:4.1f} | cmd {u.mean():+6.1f} sd {u.std():5.1f} "
            f"peak {fu:5.1f} Hz x{pu:4.1f} flips {flips:4.0f}/s | speed sd {v.std():5.1f} mm/s, mean {v.mean():+6.1f}")


def analyse_file(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        print(f"window {int(w) + 1:3d}: " + analyse({k: r[k].astype(float) for k in a.dtype.names}))


st = {"en": None, "ctrl": "?", "rows": [], "done": False}


def on_connect(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/telem"); c.subscribe("rider/debug/dcap")


def on_message(c, u, m):
    pl = m.payload.decode(errors="replace").strip()
    if m.topic == "rider/debug/telem" and pl.startswith("th="):
        d = dict(x.split("=", 1) for x in pl.split() if "=" in x)
        st["en"], st["ctrl"] = d.get("en"), d.get("ctrl", "?")
    elif pl.startswith("# dcap done"):
        st["done"] = True
    elif pl.startswith("# dcap ") and not pl.startswith("# dcap n="):
        p = pl.split()
        if len(p) == 2 + len(COLS):
            st["rows"].append([float(x) for x in p[2:]])


def main():
    if sys.argv[1:2] == ["--analyse"]:
        analyse_file(sys.argv[2])
        return
    # one recorder at a time: two would both arm the firmware's single capture buffer and take each other's samples
    # (2026-10-28: a forgotten earlier run did exactly that and garbled a recording)
    import fcntl
    lock = open("/tmp/rider_balance_watch.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another balance_watch.py is already recording -- stop it first")
    out = sys.argv[1] if len(sys.argv) > 1 else "balance_watch.csv"
    windows = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="balance_watch")
    c.on_connect = on_connect; c.on_message = on_message
    c.connect(BROKER, 1883, 30); c.loop_start()
    t0 = time.time()
    while st["en"] is None and time.time() - t0 < 10:
        time.sleep(0.1)
    if st["en"] is None:
        sys.exit("no telemetry from the robot")
    with open(out, "w", buffering=1) as f:                  # line-buffered: a stopped run keeps what it saw
        f.write("window,ctrl," + ",".join(COLS) + "\n")
        for w in range(windows):
            st["rows"] = []; st["done"] = False
            ctrl = st["ctrl"]
            c.publish("rider/control/line", json.dumps({"line": "drivecap"}), qos=1)
            time.sleep(3.3)
            c.publish("rider/control/line", json.dumps({"line": "logdump"}), qos=1)
            t1 = time.time()
            while not st["done"] and time.time() - t1 < 15:
                time.sleep(0.05)
            rows = np.array(st["rows"])
            for r in st["rows"]:
                f.write(f"{w},{ctrl}," + ",".join(str(int(x)) for x in r) + "\n")   # all integers: exact
            if len(rows):
                print(f"window {w + 1:3d}: " + analyse({k: rows[:, j] for j, k in enumerate(COLS)}, ctrl), flush=True)
            else:
                print(f"window {w + 1:3d}: no samples", flush=True)
    c.loop_stop()


if __name__ == "__main__":
    main()
