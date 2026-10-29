#!/usr/bin/env python3
"""Rider Pi bridge — LCD status screen + MQTT republish + command relay.

Single owner of /dev/ttyAMA0 (the ESP32 balance-fw line protocol: th/roll/wx/...).
Three jobs in one process (must be one — only one thing can own the serial port):

  1. LCD  : render live balance telemetry on the XGO 2-inch display.
  2. PUB  : republish telemetry over MQTT for the workstation GUI
            (rider/status + rider/status/imu), broker = local mosquitto.
  3. RELAY: subscribe to rider/control/* and forward to the ESP32 as line
            commands (rider/control/line {"line":"en 1"} -> 'en 1\\n';
            rider/control/system emergency_stop -> 'en 0').

Run:  /home/pi/xgovenv/bin/python rider_status_screen.py
"""
import time
import re
import glob
import json
import queue
import socket
import subprocess
import serial
import psutil
import lgpio
import paho.mqtt.client as mqtt
import xgoscreen.LCD_2inch as LCD_2inch
from PIL import Image, ImageDraw, ImageFont

PORT = "/dev/ttyAMA0"
FONT = "/home/pi/model/msyh.ttc"
BG = (15, 21, 46)
BROKER = "localhost"
BROKER_PORT = 1883
ROLL_MODES = {0: "off", 1: "flat", 2: "lean"}   # firmware body roll, telemetry lev (level 0/1/2)

f_s = ImageFont.truetype(FONT, 15)
f_m = ImageFont.truetype(FONT, 20)
f_l = ImageFont.truetype(FONT, 30)

lcd = LCD_2inch.LCD_2inch()
lcd.Init()
lcd.clear()
ser = serial.Serial(PORT, 115200, timeout=0.1)
# End whatever partial line the ESP32 holds: at a real power-up it has bytes from the Pi's boot on this UART
# with no newline, so the FIRST command after boot was glued onto them and rejected as unknown (2026-10-21:
# 'leghome' / 'legstep' ignored once after every power-up, the second always worked). The rejected junk comes
# back as a '# ?' line (forwarded to rider/debug/unknown, kept in riderlog).
ser.write(b"\n")

# physical buttons on the XGO 2-inch board (active-low, pull-up). Mapped BY POSITION
# on 2026-06-14 (GPIOs from the XGO CM4 key.py; labels A/B/C/D, but position is what we use):
#   upper-left  = GPIO17 (C)  -> BALANCE / STOP: idle -> 'en 1'; balancing -> STOP = 'en 0' + 'leglimp'
#                                (balance off AND legs torque-off). Merged 2026-10-19 (was two buttons).
#   upper-right = GPIO22 (D)  -> RESERVED for the hotspot on/off toggle (docs/WEB_UI_DESIGN.md); unused now
#   lower-left  = GPIO23 (B)  -> hold ~1.5s = sudo poweroff
#   lower-right = GPIO24 (A)  -> tap = reset distance frame ('poszero'; odometer + target -> 0)
BTN_BALANCE = 17                # upper-left  (C) -> balance / stop
BTN_POWER   = 23                # lower-left  (B) -> poweroff on ~1.5s hold
BTN_RESET   = 24                # lower-right (A) -> tap = reset distance ('poszero')
PWR_HOLD_S  = 1.5               # hold time before poweroff fires (also drives the LCD overlay)
try:
    _chip = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(_chip, BTN_BALANCE, lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(_chip, BTN_POWER,   lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(_chip, BTN_RESET,   lgpio.SET_PULL_UP)
    btn_ok = True
except Exception as e:
    print("button GPIO unavailable: %s" % e, flush=True)
    btn_ok = False
btn_prev = 1                    # 1 = released (pulled up)
btn_last = 0.0                  # last-press time (debounce)
pwr_down_t = 0.0                # power-button press-start time (0 = released)
pwr_fired = False               # poweroff already triggered this hold
reset_prev = 1                  # reset-distance button (lower-right) prev level
reset_last = 0.0                # reset last-press time (debounce)

tel = {}
cmd_q = queue.Queue()           # inbound MQTT -> ESP32 line commands
# forward safety governor state (rider/safety/fwd_limit): obstacle distance ahead
g_safety = {"reason": "clear", "mm": -1, "factor": 1.0, "on": False, "t": 0.0}


g_ctrl = "?"                    # active controller from telemetry: POL (policy) or LQR


def parse(line):
    for k, v in re.findall(r"(\w+)=(-?\d+(?:\.\d+)?)", line):   # strict float token: no '1.2.3' / '.' that would crash float()
        tel[k] = float(v)
    m = re.search(r"\bctrl=(\w+)", line)     # string field, skipped by the numeric regex above
    if m:
        global g_ctrl
        g_ctrl = m.group(1)


# Under-voltage alarms since the Pi booted (the 5 V lead's connector came loose once -- docs/HARDWARE.md ->
# Power). Starts from the kernel's own count for this boot, then counts rising edges of the rpi_volt alarm,
# which the kernel latches for >= 2 s per event, so the 3 Hz poll in publish() sees every one.
UV_ALARM = next((f"{h}/in0_lcrit_alarm" for h in glob.glob("/sys/class/hwmon/hwmon*")
                 if open(f"{h}/name").read().strip() == "rpi_volt"), None)
def _uv_now():
    try:
        return open(UV_ALARM).read().strip() == "1"
    except Exception:
        return False
try:
    uv_count = int(subprocess.run("journalctl -k -b --no-pager | grep -c 'Undervoltage detected'", shell=True,
                                  capture_output=True, text=True).stdout.strip() or 0)
except Exception:
    uv_count = 0
uv_prev = _uv_now()                     # an alarm already on is in the kernel count -- don't count it twice
def undervolt_alarms():
    global uv_count, uv_prev
    now = _uv_now()
    if now and not uv_prev:
        uv_count += 1
    uv_prev = now
    return uv_count


def cpu_temp_c():
    try:
        return float(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
    except Exception:
        return float("nan")


def cpu_ghz():
    """Current CPU clock (GHz). The 'ondemand' governor idles at 1.5 GHz and runs up to the
    config.txt cap (arm_freq=2000 since 2026-10-19), so 1.5 at rest is normal, not throttling."""
    try:
        return float(open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").read()) / 1e6
    except Exception:
        return float("nan")


_pw = {"t": 0.0, "w": 0.0}


def cm5_power_w():
    """Total CM5 module power (W) = sum of PMIC rail current*voltage.
    Excludes EXT5V/BATT (input rails, no paired current) so it's post-regulation
    consumption. Cached ~2 s -- vcgencmd is a subprocess."""
    now = time.time()
    if now - _pw["t"] >= 2.0:
        _pw["t"] = now
        try:
            out = subprocess.run(["vcgencmd", "pmic_read_adc"],
                                 capture_output=True, text=True, timeout=1.0).stdout
            rails = {}
            for name, kind, val in re.findall(r"(\w+)_(A|V)\s+\w+\(\d+\)=([\d.]+)", out):
                rails.setdefault(name, {})[kind] = float(val)
            _pw["w"] = sum(r["A"] * r["V"] for r in rails.values() if "A" in r and "V" in r)
        except Exception:
            pass
    return _pw["w"]


_ctrl = {"t": 0.0, "on": False}


def controller_connected():
    """DS4-over-BT presence via BlueZ link state. 'bluetoothctl devices Connected'
    flips to disconnected as soon as the BT stack sees the drop -- ahead of the
    /dev/input/js* node teardown, which lagged. Cached at ~2 Hz (subprocess)."""
    now = time.time()
    if now - _ctrl["t"] >= 0.5:
        _ctrl["t"] = now
        try:
            out = subprocess.run(["bluetoothctl", "devices", "Connected"],
                                 capture_output=True, text=True, timeout=1.0).stdout
            _ctrl["on"] = "Controller" in out          # DS4 advertises as "Wireless Controller"
        except Exception:
            _ctrl["on"] = bool(glob.glob("/dev/input/js*"))   # fallback if bluetoothctl unavailable
    return _ctrl["on"]


def controller_battery():
    """The DS4's own battery (kernel hid-playstation: /sys/class/power_supply/ps-controller-battery-<mac>) as
    (percent, charging) -- (None, False) when no controller is attached."""
    for d in glob.glob("/sys/class/power_supply/ps-controller-battery-*"):
        try:
            with open(d + "/capacity") as f:
                pct = int(f.read().strip())
            with open(d + "/status") as f:
                st = f.read().strip()
            return pct, st in ("Charging", "Full")
        except (OSError, ValueError):
            pass
    return None, False


g_power_msg = None              # set once a poweroff/reboot fires -> render() shows only this from then on


def do_poweroff():
    """Shut the Pi down (pi has NOPASSWD sudo). Triggered by the lower-left button
    held ~1.5s, or the GUI's rider/control/system 'poweroff_pi' command."""
    global g_power_msg
    g_power_msg = "SHUTTING DOWN"   # next frame (<=0.2 s) switches the LCD to it; it stays until power drops
    print("POWEROFF requested -> sudo poweroff", flush=True)
    try:
        subprocess.Popen(["sudo", "poweroff"])
    except Exception as e:
        print("poweroff failed: %s" % e, flush=True)


def do_reboot():
    """Reboot the Pi (pi has NOPASSWD sudo). Triggered by the GUI's
    rider/control/system 'reboot_pi' command (mirrors do_poweroff)."""
    global g_power_msg
    g_power_msg = "REBOOTING"
    print("REBOOT requested -> sudo reboot", flush=True)
    try:
        subprocess.Popen(["sudo", "reboot"])
    except Exception as e:
        print("reboot failed: %s" % e, flush=True)


# ---------------- MQTT (republish + relay) ----------------
g_drivemode = "computer"   # joystick / computer -- boot default is computer; controller publishes on change
g_wifi_state = None       # rider/wifi (the WiFi watchdog, which pings the router): "up" / "down" / "starting"

def on_connect(client, userdata, flags, reason_code, properties=None):
    for t in ("rider/control/line", "rider/control/system",
              "rider/control/movement", "rider/control/settings", "rider/control/drivemode",
              "rider/safety/fwd_limit", "rider/wifi"):
        client.subscribe(t)


def on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
    except Exception:
        p = {}
    t = msg.topic
    if t == "rider/safety/fwd_limit":
        try:                                             # tolerate a malformed/null field w/o dropping the whole msg
            g_safety["reason"] = str(p.get("reason", "clear"))
            g_safety["mm"] = int(p.get("min_mm", -1))
            g_safety["factor"] = float(p.get("factor", 1.0))
            g_safety["on"] = bool(p.get("on", False))      # limiting switched on (web page); off = report only
            g_safety["t"] = time.time()
        except (TypeError, ValueError):
            pass
        return
    if t == "rider/wifi":
        global g_wifi_state
        g_wifi_state = p.get("state")
        return
    if t == "rider/control/drivemode":
        global g_drivemode
        m = (p.get("mode") or "").lower()
        if m in ("joystick", "computer"):
            g_drivemode = m
    elif t == "rider/control/line":
        line = p.get("line") or p.get("cmd")
        if line:
            cmd_q.put(str(line))
    elif t == "rider/control/system":
        c = (p.get("command") or p.get("action") or "").lower()
        if c in ("emergency_stop", "stop", "disable"):
            cmd_q.put("en 0")
        elif c in ("enable", "balance_on"):
            cmd_q.put("en 1")
        elif c in ("poweroff_pi", "poweroff", "shutdown"):   # GUI "POWER OFF PI" button
            do_poweroff()
        elif c in ("reboot_pi", "reboot", "restart"):        # GUI "REBOOT PI" button
            do_reboot()
    # movement/settings: logged only for now (full mapping is the GUI-reconcile step)
    else:
        print("relay: unmapped %s %s" % (t, p), flush=True)


mqc = None
try:
    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_bridge")
    mqc.on_connect = on_connect
    mqc.on_message = on_message
    mqc.will_set("rider/client/disconnect",
                 json.dumps({"source": "rider_bridge"}), retain=False)
    mqc.connect(BROKER, BROKER_PORT, keepalive=30)
    mqc.loop_start()
except Exception as e:
    print("MQTT unavailable (%s) -- screen-only" % e, flush=True)
    mqc = None


def publish():
    if mqc is None:
        return
    en = int(tel.get("en", 0))
    th = tel.get("th", 0.0); roll = tel.get("roll", 0.0); yaw = tel.get("yaw", 0.0)
    batt = int(tel.get("batt", 0)); vbat = round(tel.get("vbat", 0.0), 2)
    on = controller_connected()
    cpct, cchg = controller_battery() if on else (None, False)
    status = {
        "controller_connected": on,                        # real DS4-over-BT state (was hardcoded True)
        "controller_battery": cpct,                        # DS4 battery %, None when not connected
        "controller_charging": cchg,
        "connection_status": "connected",
        "roll_balance_enabled": bool(en),   # = BALANCING on (legacy XGO name; the DS4 script reads it as such)
        "roll_mode": ROLL_MODES.get(int(tel.get("lev", 2)), "lean"),   # firmware body roll (telemetry lev 0/1/2)
        "battery_level": batt,
        "battery_voltage": vbat,
        "cpu_percent": psutil.cpu_percent(),
        "cpu_temp": round(cpu_temp_c(), 1),
        "power_w": round(cm5_power_w(), 2),
        "loop_hz": int(tel.get("lhz", 0)),
        "fault": int(tel.get("fault", 0)),
        "undervolt_alarms": undervolt_alarms(),   # Pi 5 V under-voltage alarms since the Pi booted
        "read_fails": int(tel.get("rfails", 0)),   # failed wheel reads since the ESP32 booted (count)
        "mode": g_ctrl,                 # active controller ("POL"/"LQR"), parsed from telem ctrl= (firmware 'ctrl 0|1')
        "position": tel.get("wx", 0.0),
        "target": tel.get("ptgt", 0.0),
        "pose_x": round(tel.get("px", 0.0), 3),
        "pose_y": round(tel.get("py", 0.0), 3),
        # NOTE: roll/pitch/yaw (incl. heading) are published ONLY on rider/status/imu
        # and the odom topic, not here. Duplicating them into rider/status makes the
        # GUI's update_status() write them first, which starves update_imu()'s
        # change-gate -> IMU panel never fires.
    }
    try:
        mqc.publish("rider/status", json.dumps(status))
        mqc.publish("rider/status/imu",
                    json.dumps({"roll": roll, "pitch": th, "yaw": yaw}))
        mqc.publish("rider/status/battery",
                    json.dumps({"level": batt, "voltage": vbat}))
        # dead-reckoned pose: world-frame (x,y) + heading. Foundation for waypoints /
        # return-to-start. (px,py) integrate fused wheel distance through gyro heading.
        mqc.publish("rider/status/odom",
                    json.dumps({"x": round(tel.get("px", 0.0), 3),
                                "y": round(tel.get("py", 0.0), 3),
                                "heading": round(yaw, 1)}))
    except Exception:
        pass


# ---------------- LCD render ----------------
def _text_w(d, s, font):
    """text width, compatible across Pillow versions."""
    try:
        b = d.textbbox((0, 0), s, font=font); return b[2] - b[0]
    except AttributeError:
        return d.textsize(s, font=font)[0]


def draw_battery(d, x, y, pct, font, w=50, h=26):
    """iPhone-style battery: rounded body + terminal nub, fill bar proportional to
    pct, percentage centered inside. Color goes green->amber->red as it drains."""
    pct = max(0, min(100, int(pct)))
    col = (0, 255, 140) if pct > 40 else (255, 200, 70) if pct > 15 else (255, 90, 90)
    bx0, by0, bx1, by1 = x, y, x + w, y + h
    rr = getattr(d, "rounded_rectangle", None)   # Pillow >= 8.2; fall back to square corners
    if rr:
        rr((bx0, by0, bx1, by1), radius=5, outline=col, width=2)
    else:
        d.rectangle((bx0, by0, bx1, by1), outline=col, width=2)
    d.rectangle((bx1 + 2, y + 7, bx1 + 5, by1 - 7), fill=col)   # terminal nub
    pad = 3                                                     # inset from the body wall
    ix0, iy0, ix1, iy1 = bx0 + pad, by0 + pad, bx1 - pad, by1 - pad
    grey = (120, 120, 120)                                     # empty space = grey
    if rr:
        rr((ix0, iy0, ix1, iy1), radius=2, fill=grey)
    else:
        d.rectangle((ix0, iy0, ix1, iy1), fill=grey)
    fillw = int((ix1 - ix0) * pct / 100.0)                     # colored bar over the left portion
    if fillw > 0:
        if rr:
            rr((ix0, iy0, ix0 + fillw, iy1), radius=2, fill=col)
        else:
            d.rectangle((ix0, iy0, ix0 + fillw, iy1), fill=col)
    s = "%d%%" % pct                                           # % centered inside, black
    d.text((bx0 + (w - _text_w(d, s, font)) // 2, by0 + 4), s, fill=(0, 0, 0), font=font)


def wifi_connected():
    """True if a wlan interface is operationally up (associated). NOTE: iwgetid/`iw` report
    nothing on this Pi's brcmfmac driver, but /sys operstate is reliable. Fast file read, no
    subprocess -- safe to call every render."""
    try:
        for p in glob.glob("/sys/class/net/wl*/operstate"):
            if open(p).read().strip() == "up":
                return True
    except Exception:
        pass
    return False


def wifi_ip():
    """The Pi's IPv4 address on wlan0, or 'no WiFi'. psutil reads the kernel's table -- no subprocess."""
    try:
        for a in psutil.net_if_addrs().get("wlan0", []):
            if a.family == socket.AF_INET:
                return a.address
    except Exception:
        pass
    return "no WiFi"


def render():
    img = Image.new("RGB", (320, 240), BG)
    d = ImageDraw.Draw(img)
    if g_power_msg:                     # shutdown/reboot under way: a full-screen notice, nothing else
        d.rectangle((0, 0, 320, 240), fill=(140, 0, 0))
        d.text(((320 - _text_w(d, g_power_msg, f_l)) // 2, 85), g_power_msg, fill=(255, 255, 255), font=f_l)
        d.text(((320 - _text_w(d, "please wait", f_m)) // 2, 135), "please wait", fill=(255, 210, 210), font=f_m)
        lcd.ShowImage(img)
        return
    en = int(tel.get("en", 0))

    # title turns blue when a controller (DS4) is connected, white otherwise
    d.text((10, 6), "RIDER",
           fill=(80, 160, 255) if controller_connected() else (255, 255, 255), font=f_l)
    # WiFi dot (next to RIDER) -- the "can I reach the Pi" signal. It follows the WiFi watchdog's router
    # ping (green = answering, amber = starting up, grey = no answer for 15 s), since "associated" stayed
    # true through the silent dropout of 2026-10-21; without a watchdog message yet, the link state.
    if g_wifi_state is None:
        dot = (0, 200, 120) if wifi_connected() else (110, 110, 110)
    else:
        dot = {"up": (0, 200, 120), "starting": (255, 170, 60)}.get(g_wifi_state, (110, 110, 110))
    d.ellipse((92, 18, 104, 30), fill=dot)
    # battery (header, between title and badge): iPhone-style icon, % inside, no voltage
    batt = int(tel.get("batt", 0))
    draw_battery(d, 112, 11, batt, f_s)
    state = "BALANCING" if en else "IDLE"
    col = (0, 255, 140) if en else (150, 150, 150)
    d.rectangle((176, 10, 312, 44), outline=col, width=2)
    d.text((188, 14), state, fill=col, font=f_m)

    th = tel.get("th", 0.0); roll = tel.get("roll", 0.0); yaw = tel.get("yaw", 0.0)
    lev = int(tel.get("lev", 2))
    for x, lbl, val in ((10, "tilt", th), (112, "roll", roll), (214, "yaw", yaw)):
        d.text((x, 52), lbl, fill=(150, 165, 205), font=f_s)
        # the legs working the roll (FLAT or LEAN): the roll value green + the mode's tag by its header
        rcol = (0, 255, 140) if (lbl == "roll" and lev) else (255, 255, 255)
        d.text((x, 70), "%+.1f°" % val, fill=rcol, font=f_m)
    if lev:
        d.text((112 + _text_w(d, "roll", f_s) + 6, 52), "FLAT" if lev == 1 else "LEAN", fill=(0, 255, 140), font=f_s)

    # forward path (ToF governor): OBSTACLE distance ahead, or CLEAR. Red = stopped
    # (factor 0, <=~10cm), amber = slowing, green = clear, grey = governor offline (stale).
    # Limiting switched OFF (web page): the distance still shows, in grey, prefixed "off".
    sf = g_safety
    if time.time() - sf["t"] > 3.0:
        s_txt, s_col = "FWD --", (110, 110, 110)                            # governor offline
    elif not sf["on"]:
        obst = sf["reason"] == "obstacle" and sf["mm"] >= 0
        s_txt = "ToF off" + (" OBST %dcm" % round(sf["mm"] / 10.0) if obst else "")
        s_col = (110, 110, 110)
    elif sf["reason"] == "obstacle" and sf["mm"] >= 0:
        # green = clear ahead, amber = slowing, red = stopped (<=~10cm)
        s_col = (255, 90, 90) if sf["factor"] <= 0.001 else \
                (255, 170, 60) if sf["factor"] < 0.999 else (0, 255, 140)
        s_txt = "OBSTACLE %dcm" % round(sf["mm"] / 10.0)
    else:
        s_txt, s_col = "---", (110, 110, 110)                               # nothing within range
    d.text((10, 96), s_txt, fill=s_col, font=f_m)

    # position (wheel travel since balance-on, wx) and the dead-reckoned pose x, y (px, py) -- the web page's
    # POSE numbers -- laid out like the tilt/roll/yaw row
    for x, lbl, val, fmt in ((10, "position", tel.get("wx", 0.0), "%+.3fm"), (112, "pose x", tel.get("px", 0.0), "%+.2fm"),
                             (214, "pose y", tel.get("py", 0.0), "%+.2fm")):
        d.text((x, 122), lbl, fill=(150, 165, 205), font=f_s)
        d.text((x, 140), fmt % val, fill=(255, 255, 255), font=f_m)

    # bottom line, just above the CPU row: drive source (Start-button toggle; cyan = JOYSTICK, orange = COMPUTER)
    # left, active controller (POLICY green / LQR purple) right, and between them the Pi's WiFi address --
    # where a phone finds the web page (home 10.0.0.x, phone hotspot 172.20.10.x)
    dm_txt = g_drivemode.upper()
    dm_col = (80, 200, 255) if g_drivemode == "joystick" else (255, 170, 60)
    d.text((10, 186), dm_txt, fill=dm_col, font=f_m)
    ctrl_disp = {"POL": "POLICY", "LQR": "LQR"}.get(g_ctrl, g_ctrl)
    ctrl_col = (150, 220, 150) if g_ctrl == "POL" else (190, 170, 255)
    ctrl_x = 312 - _text_w(d, ctrl_disp, f_m)
    d.text((ctrl_x, 186), ctrl_disp, fill=ctrl_col, font=f_m)
    ip = wifi_ip()
    gap_l = 10 + _text_w(d, dm_txt, f_m)
    d.text((int((gap_l + ctrl_x - _text_w(d, ip, f_s)) / 2), 190), ip, fill=(150, 165, 205), font=f_s)

    cpu = psutil.cpu_percent(); temp = cpu_temp_c(); pw = cm5_power_w(); ghz = cpu_ghz()
    dim = (170, 195, 235)
    tcol = (255, 90, 90) if temp >= 80 else (255, 215, 60) if temp >= 70 else dim
    # four items -- CPU load, temperature, power, clock -- spread evenly across x=10..310
    items = [("CPU %d%%" % cpu, dim), ("%.0f°C" % temp, tcol), ("%.1fW" % pw, dim), ("%.1fGHz" % ghz, dim)]
    widths = [_text_w(d, s, f_m) for s, _ in items]
    gap = (300 - sum(widths)) / (len(items) - 1)
    x = 10.0
    for (s, col), w in zip(items, widths):
        d.text((int(x), 214), s, fill=col, font=f_m)
        x += w + gap
    # power-button hold overlay: warn + show progress over the hold; release cancels
    if pwr_down_t > 0.0:
        held = min(time.time() - pwr_down_t, PWR_HOLD_S)
        d.rectangle((0, 0, 320, 240), fill=(140, 0, 0))
        d.text((44, 60), "POWERING OFF", fill=(255, 255, 255), font=f_l)
        d.text((66, 102), "release to cancel", fill=(255, 210, 210), font=f_m)
        d.rectangle((40, 150, 280, 178), outline=(255, 255, 255), width=2)
        d.rectangle((43, 153, 43 + int(234 * held / PWR_HOLD_S), 175), fill=(255, 255, 255))
    lcd.ShowImage(img)


last_render = 0.0
def mqtt_forward(topic, line):
    """Republish a raw ESP32 serial line to MQTT so it's reachable while the robot
    drives untethered (USB-C unplugged). Best-effort: a broker hiccup never stalls the bridge."""
    if mqc is not None:
        try:
            mqc.publish(topic, line)
        except Exception:
            pass

last_pub = 0.0
psutil.cpu_percent()  # prime
while True:
    line = ser.readline().decode(errors="replace").strip()
    if line.startswith("th="):
        parse(line)
        mqtt_forward("rider/debug/telem", line)   # high-rate raw telemetry (~ESP32 rate)
    elif line.startswith("# dcap"):
        mqtt_forward("rider/debug/dcap", line)     # drive-capture dump (commanded-vs-actual wheel vel)
    elif line.startswith("# cfg"):
        mqtt_forward("rider/debug/cfg", line)      # servo config-register dump ('cfgdump <id>')
    elif line.startswith("# leg"):
        mqtt_forward("rider/debug/leg", line)      # leg arrival check after a slow leg move (ok / STUCK + registers)
    elif line.startswith("# ?"):
        mqtt_forward("rider/debug/unknown", line)  # a command line the firmware did not recognise
    # relay any queued commands to the ESP32
    while not cmd_q.empty():
        try:
            ser.write((cmd_q.get_nowait() + "\n").encode())
        except Exception:
            break
    # C button (upper-left, GPIO17): BALANCE / STOP. Idle -> start balancing. Balancing -> STOP:
    # 'en 0' (wheels off) + 'leglimp' (legs torque-off too), so the robot settles fully limp.
    if btn_ok:
        lvl = lgpio.gpio_read(_chip, BTN_BALANCE)
        tnow = time.time()
        if btn_prev == 1 and lvl == 0 and tnow - btn_last > 0.4:   # falling edge + debounce
            btn_last = tnow
            if int(tel.get("en", 0)) == 1:
                ser.write(b"en 0\n")
                ser.write(b"leglimp\n")
                print("STOP requested (en 0 + leglimp)", flush=True)
            else:
                ser.write(b"en 1\n")                               # idle -> balance (balance-on re-energizes the legs)
        btn_prev = lvl
        # power button (lower-left, GPIO23): HOLD ~1.5s -> sudo poweroff (hold, not tap,
        # so a stray press can't shut the robot down mid-use).
        if lgpio.gpio_read(_chip, BTN_POWER) == 0:
            if pwr_down_t == 0.0:
                pwr_down_t = tnow
            elif not pwr_fired and tnow - pwr_down_t >= PWR_HOLD_S:
                pwr_fired = True
                do_poweroff()
        else:
            pwr_down_t = 0.0
            pwr_fired = False
        # reset-distance button (lower-right, GPIO24): tap -> 'poszero' (zero odometer + target,
        # WITHOUT dropping balance). gcal moved off the buttons -- robot no longer recals per session.
        glvl = lgpio.gpio_read(_chip, BTN_RESET)
        if reset_prev == 1 and glvl == 0 and tnow - reset_last > 0.4:   # falling edge + debounce
            reset_last = tnow
            ser.write(b"poszero\n")
            print("DIST RESET requested (poszero)", flush=True)
        reset_prev = glvl
    now = time.time()
    if now - last_render >= 0.2:        # LCD ~5 Hz
        last_render = now
        render()
    if now - last_pub >= 0.3:           # MQTT ~3 Hz
        last_pub = now
        publish()
