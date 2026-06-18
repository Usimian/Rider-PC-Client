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

f_s = ImageFont.truetype(FONT, 15)
f_m = ImageFont.truetype(FONT, 20)
f_xl = ImageFont.truetype(FONT, 25)
f_l = ImageFont.truetype(FONT, 30)

lcd = LCD_2inch.LCD_2inch()
lcd.Init()
lcd.clear()
ser = serial.Serial(PORT, 115200, timeout=0.1)

# physical buttons on the XGO 2-inch board (active-low, pull-up). Mapped BY POSITION
# on 2026-06-14 (GPIOs from the XGO CM4 key.py; labels A/B/C/D, but position is what we use):
#   upper-left  = GPIO17 (C)  -> balance start/stop toggle (next to the RIDER text)
#   upper-right = GPIO22 (D)  -> tap = STOP (en 0) to the ESP32
#   lower-left  = GPIO23 (B)  -> hold ~1.5s = sudo poweroff
#   lower-right = GPIO24 (A)  -> tap = reset distance frame ('poszero'; odometer + target -> 0)
BTN_BALANCE = 17                # upper-left  (C)
BTN_POWER   = 23                # lower-left  (B) -> poweroff on ~1.5s hold
BTN_RESET   = 24                # lower-right (A) -> tap = reset distance ('poszero')
BTN_STOP    = 22                # upper-right (D) -> tap = STOP (en 0) to the ESP32
PWR_HOLD_S  = 1.5               # hold time before poweroff fires (also drives the LCD overlay)
try:
    _chip = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(_chip, BTN_BALANCE, lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(_chip, BTN_POWER,   lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(_chip, BTN_RESET,   lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(_chip, BTN_STOP,    lgpio.SET_PULL_UP)
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
stop_prev = 1                   # stop button (upper-right) prev level
stop_last = 0.0                 # stop last-press time (debounce)

tel = {}
cmd_q = queue.Queue()           # inbound MQTT -> ESP32 line commands


def parse(line):
    for k, v in re.findall(r"(\w+)=(-?[\d.]+)", line):
        tel[k] = float(v)


def cpu_temp_c():
    try:
        return float(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
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


def do_poweroff():
    """Shut the Pi down (pi has NOPASSWD sudo). Triggered by the lower-left button
    held ~1.5s, or the GUI's rider/control/system 'poweroff_pi' command."""
    print("POWEROFF requested -> sudo poweroff", flush=True)
    try:
        subprocess.Popen(["sudo", "poweroff"])
    except Exception as e:
        print("poweroff failed: %s" % e, flush=True)


def do_reboot():
    """Reboot the Pi (pi has NOPASSWD sudo). Triggered by the GUI's
    rider/control/system 'reboot_pi' command (mirrors do_poweroff)."""
    print("REBOOT requested -> sudo reboot", flush=True)
    try:
        subprocess.Popen(["sudo", "reboot"])
    except Exception as e:
        print("reboot failed: %s" % e, flush=True)


# ---------------- MQTT (republish + relay) ----------------
def on_connect(client, userdata, flags, reason_code, properties=None):
    for t in ("rider/control/line", "rider/control/system",
              "rider/control/movement", "rider/control/settings"):
        client.subscribe(t)


def on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
    except Exception:
        p = {}
    t = msg.topic
    if t == "rider/control/line":
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
    en = int(tel.get("en", 0)); pol = int(tel.get("polrun", 0))
    th = tel.get("th", 0.0); roll = tel.get("roll", 0.0); yaw = tel.get("yaw", 0.0)
    batt = int(tel.get("batt", 0)); vbat = round(tel.get("vbat", 0.0), 2)
    status = {
        "controller_connected": controller_connected(),   # real DS4-over-BT state (was hardcoded True)
        "connection_status": "connected",
        "roll_balance_enabled": bool(en),
        "battery_level": batt,
        "battery_voltage": vbat,
        "cpu_percent": psutil.cpu_percent(),
        "cpu_temp": round(cpu_temp_c(), 1),
        "power_w": round(cm5_power_w(), 2),
        "loop_hz": int(tel.get("lhz", 0)),
        "fault": int(tel.get("fault", 0)),
        "rfail": tel.get("rfail", 0.0),
        "mode": "policy" if pol else "pid",
        "position": tel.get("wx", 0.0),
        "target": tel.get("ptgt", 0.0),
        "wheel_l": tel.get("wp1", 0.0),
        "wheel_r": tel.get("wp2", 0.0),
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


def render():
    img = Image.new("RGB", (320, 240), BG)
    d = ImageDraw.Draw(img)
    en = int(tel.get("en", 0))
    pol = int(tel.get("polrun", 0))
    mq_ok = mqc is not None and mqc.is_connected()

    # title turns blue when a controller (DS4) is connected, white otherwise
    d.text((10, 6), "RIDER",
           fill=(80, 160, 255) if controller_connected() else (255, 255, 255), font=f_l)
    # WiFi link dot (next to RIDER): green = wlan associated, grey = down -- the "can I reach the Pi" signal
    d.ellipse((92, 18, 104, 30), fill=(0, 200, 120) if wifi_connected() else (110, 110, 110))
    # battery (header, between title and badge): iPhone-style icon, % inside, no voltage
    batt = int(tel.get("batt", 0))
    draw_battery(d, 112, 11, batt, f_s)
    state = "BALANCING" if en else "IDLE"
    col = (0, 255, 140) if en else (150, 150, 150)
    d.rectangle((176, 10, 312, 44), outline=col, width=2)
    d.text((188, 14), state, fill=col, font=f_m)

    th = tel.get("th", 0.0); roll = tel.get("roll", 0.0); yaw = tel.get("yaw", 0.0)
    for x, lbl, val in ((10, "tilt", th), (112, "roll", roll), (214, "yaw", yaw)):
        d.text((x, 52), lbl, fill=(150, 165, 205), font=f_s)
        d.text((x, 70), "%+.1f°" % val, fill=(255, 255, 255), font=f_m)

    wp1 = tel.get("wp1", 0.0); wp2 = tel.get("wp2", 0.0)
    wx = tel.get("wx", 0.0); tg = tel.get("ptgt", 0.0); err = wx - tg
    ecol = (255, 200, 70) if abs(err) > 0.030 else (0, 255, 140)
    # R = robot's right wheel (wp1), L = wp2  -- swapped so R sits on the right
    d.text((10, 112), "L %+.3fm    R %+.3fm" % (wp2, wp1), fill=(255, 255, 255), font=f_xl)
    d.text((10, 148), "Tgt %+.3fm    Err %+.3fm" % (tg, err), fill=ecol, font=f_xl)

    cpu = psutil.cpu_percent(); temp = cpu_temp_c(); pw = cm5_power_w()
    tcol = (255, 90, 90) if temp >= 80 else (255, 215, 60) if temp >= 70 else (170, 195, 235)
    d.text((10, 214), "CPU %d%%" % cpu, fill=(170, 195, 235), font=f_m)
    d.text((140, 214), "%.0f°C" % temp, fill=tcol, font=f_m)
    d.text((232, 214), "%.1fW" % pw, fill=(170, 195, 235), font=f_m)
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
last_pub = 0.0
psutil.cpu_percent()  # prime
while True:
    line = ser.readline().decode(errors="replace").strip()
    if line.startswith("th="):
        parse(line)
        # high-rate raw-telemetry republish for untethered diagnostics (~ESP32 rate)
        if mqc is not None:
            try:
                mqc.publish("rider/debug/telem", line)
            except Exception:
                pass
    elif line.startswith("# dcap"):
        # forward the firmware's drive-capture dump (commanded-vs-actual wheel vel)
        # so it's reachable over MQTT while the robot drives untethered (USB-C unplugged).
        if mqc is not None:
            try:
                mqc.publish("rider/debug/dcap", line)
            except Exception:
                pass
    elif line.startswith("# cfg"):
        # forward the firmware's servo config-register dump ('cfgdump <id>') over MQTT
        # so wheel-servo registers can be inspected/compared without USB-C / passthrough fw.
        if mqc is not None:
            try:
                mqc.publish("rider/debug/cfg", line)
            except Exception:
                pass
    # relay any queued commands to the ESP32
    while not cmd_q.empty():
        try:
            ser.write((cmd_q.get_nowait() + "\n").encode())
        except Exception:
            break
    # C button (upper-left, GPIO17): toggle the balance policy on each press
    if btn_ok:
        lvl = lgpio.gpio_read(_chip, BTN_BALANCE)
        tnow = time.time()
        if btn_prev == 1 and lvl == 0 and tnow - btn_last > 0.4:   # falling edge + debounce
            btn_last = tnow
            if int(tel.get("en", 0)) == 1:
                ser.write(b"en 0\n")                                # balancing -> stop
            else:
                ser.write(b"polrun 1\nen 1\n")                      # idle -> arm policy + enable
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
        # STOP button (upper-right, GPIO22): tap -> immediate 'en 0' to the ESP32 (estop)
        slvl = lgpio.gpio_read(_chip, BTN_STOP)
        if stop_prev == 1 and slvl == 0 and tnow - stop_last > 0.4:   # falling edge + debounce
            stop_last = tnow
            ser.write(b"en 0\n")
            print("STOP requested (en 0)", flush=True)
        stop_prev = slvl
    now = time.time()
    if now - last_render >= 0.2:        # LCD ~5 Hz
        last_render = now
        render()
    if now - last_pub >= 0.3:           # MQTT ~3 Hz
        last_pub = now
        publish()
