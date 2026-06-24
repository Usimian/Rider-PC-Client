#!/usr/bin/env python3
"""Rider DS4 controller -> MQTT command publisher (runs on the Pi).

Reads a paired PS4 DualShock 4 via pygame and publishes ESP32 line commands to
rider/control/line, which the bridge (rider_status_screen.py) relays to the ESP32.
Kept separate from the bridge so a controller hiccup can't disturb the balance
serial/telemetry -- the bridge stays the single serial owner.

Mapping (balancer w/ position-hold; turning not available yet):
  Left stick up/down -> drive the position target (push = move, release = hold)
  Triangle           -> legs EXTEND (body up), stepped, firmware clamps to servo limits
  Cross (X)          -> legs RETRACT (body down), stepped, firmware clamps to servo limits
  Circle (O)         -> toggle balance (polrun 1 + en 1 / en 0); off = stop (no separate e-stop)
  Square (#)         -> zero the distance frame (poszero) without dropping balance

Run normally:  /home/pi/xgovenv/bin/python rider_controller.py
Verify mapping: /home/pi/xgovenv/bin/python rider_controller.py --test
"""
import os, sys, time, json, signal, threading
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
import pygame
import paho.mqtt.client as mqtt

_running = True


def _shutdown(*_a):
    global _running
    _running = False

BROKER, PORT = "localhost", 1883
TOPIC_CMD, TOPIC_STATUS = "rider/control/line", "rider/status"

# --- mapping (verify indices with --test, then adjust) ---
AXIS_DRIVE = 1       # left stick Y (stick up = negative)
AXIS_TURN = 3        # right stick X (verify with --test)
BTN_BALANCE     = 1  # Circle (O)  -- toggle balance (moved here from Cross 2026-06-24; off = stop)
BTN_LEG_EXTEND  = 2  # Triangle    -- legs extend (body up)
BTN_LEG_RETRACT = 0  # Cross (X)   -- legs retract (body down)
BTN_DISTZERO    = 3  # Square       -- poszero (Cross=0,Circle=1,Triangle=2,Square=3 verified --test 2026-06-16)
BTN_MODE        = 9  # Start/Options -- toggles joystick<->computer drive
LEG_STEP        = 10 # encoder counts per Triangle/Cross press (leg-goal nudge; firmware clamps to servo limits)
DEADZONE = 0.12
MAX_SPEED = 0.25     # m/s at full stick. ->0.25 (2026-06-19, LQR): matches firmware gPosVmax now that the dv-advance
                     # fix + LQR drive the robot at real speed. Drive authority tops ~0.15 m/s; cap leaves headroom.
MAX_YAW_RATE = 1.0   # rad/s commanded at full right-stick (firmware closes the loop on the gyro)
TURN_SIGN = 1        # flip if right-stick-right turns the wrong way
SEND_HZ = 20
LOOP_HZ = 50

BEEP_WAV = "/home/pi/beep.wav"
_beep_snd = None     # pygame.mixer.Sound, loaded in main() after pygame.init() (which opens the audio device)
def beep(n):
    """n short beeps, non-blocking: 1 = joystick mode, 2 = computer mode. Plays through pygame's
    own mixer -- pygame.init() already holds the audio device, so no separate aplay / ALSA card to
    fight over (the card-number reshuffle that broke the old aplay path can't bite here)."""
    if _beep_snd is None:
        return
    def _play():
        for _ in range(n):
            _beep_snd.play()
            time.sleep(0.18)
    threading.Thread(target=_play, daemon=True).start()

state = {"en": 0, "pos": 0.0}


def on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
    except Exception:
        return
    if "roll_balance_enabled" in p:
        state["en"] = 1 if p["roll_balance_enabled"] else 0
    if "position" in p:
        try:
            state["pos"] = float(p["position"])
        except Exception:
            pass


def main():
    test = "--test" in sys.argv
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_controller")
    mqc.on_message = on_message
    # subscribe in on_connect so the balance-state sync (-> drive gate) survives the
    # initial CONNACK race AND any reconnect. A subscribe before loop_start/CONNACK is
    # dropped by the broker -> state["en"] stuck 0 -> drive/turn silently suppressed.
    def _on_connect(c, u, flags, rc, properties=None):
        c.subscribe(TOPIC_STATUS)
    mqc.on_connect = _on_connect
    mqc.reconnect_delay_set(min_delay=1, max_delay=10)
    mqc.connect(BROKER, PORT, keepalive=30)
    mqc.loop_start()

    def send(line):
        mqc.publish(TOPIC_CMD, json.dumps({"line": line}), qos=0)

    def pub_mode(m):
        mqc.publish("rider/control/drivemode", json.dumps({"mode": m}), qos=0, retain=True)

    pygame.init()
    pygame.joystick.init()
    global _beep_snd
    try:
        if not pygame.mixer.get_init():
            pygame.mixer.init()
        _beep_snd = pygame.mixer.Sound(BEEP_WAV)
    except Exception as e:
        print("beep audio unavailable:", e, flush=True)
    js = None

    def ensure_js():
        nonlocal js
        if pygame.joystick.get_count() > 0:
            js = pygame.joystick.Joystick(0); js.init()
            print("joystick:", js.get_name(), "axes", js.get_numaxes(), "buttons", js.get_numbuttons(), flush=True)
            return True
        js = None
        return False

    last_vel = None
    prev_btn = {}
    last_send = 0.0
    last_turn = None
    last_turn_send = 0.0
    drive_mode = "computer"   # BOOT in computer mode (safe default). 'joystick' = stick drives; press Start to opt in.
    period = 1.0 / LOOP_HZ
    send_period = 1.0 / SEND_HZ

    def dz(v):                       # deadzone + rescale to full range
        if abs(v) < DEADZONE:
            return 0.0
        return (v - (DEADZONE if v > 0 else -DEADZONE)) / (1 - DEADZONE)

    time.sleep(0.5); pub_mode(drive_mode)   # boot in computer mode -> tell the LCD

    while _running:
        # process the event queue: keeps axis/button state fresh AND handles hotplug --
        # when the controller sleeps/wakes/recharges, REMOVED/ADDED fire and we rebind to
        # the live device instead of clinging to a dead js0 handle.
        for ev in pygame.event.get():
            if ev.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                js = None
        if js is None:
            if not ensure_js():
                if drive_mode == "joystick":          # joystick gone -> fall back to computer
                    drive_mode = "computer"; pub_mode(drive_mode); beep(2)
                    print("joystick disconnected -> computer mode", flush=True)
                time.sleep(1.0); continue
        try:
            naxes, nbtn = js.get_numaxes(), js.get_numbuttons()
            axes = [js.get_axis(i) for i in range(naxes)]
            buttons = [js.get_button(i) for i in range(nbtn)]
        except pygame.error:
            print("joystick lost; waiting for reconnect", flush=True)
            js = None; continue

        if test:
            pressed = [i for i, b in enumerate(buttons) if b]
            print("axes:", [round(a, 2) for a in axes], " buttons pressed:", pressed, flush=True)
            time.sleep(0.15); continue

        now = time.time()

        def edge(idx):
            cur = buttons[idx] if idx < len(buttons) else 0
            return cur and not prev_btn.get(idx, 0)

        if edge(BTN_BALANCE):                  # Circle: balance on/off (toggling off = stop)
            if state["en"]:
                send("en 0")
            else:
                send("polrun 1"); send("en 1")     # firmware homes the target on enable
        if edge(BTN_LEG_EXTEND):               # Triangle: legs extend (body up)
            send("legstep %d" % LEG_STEP)
        if edge(BTN_LEG_RETRACT):              # Cross: legs retract (body down)
            send("legstep %d" % -LEG_STEP)
        if edge(BTN_DISTZERO):                 # Square: re-zero distance, balance stays on
            send("poszero")
        if edge(BTN_MODE):                     # Start/Options toggles drive source
            drive_mode = "computer" if drive_mode == "joystick" else "joystick"
            send("dv 0"); send("turnrate 0")   # clear any stick command on the switch
            beep(2 if drive_mode == "computer" else 1)   # 1 beep=joystick, 2=computer
            pub_mode(drive_mode)               # -> LCD shows the active drive source
            print("drive_mode -> %s" % drive_mode, flush=True)
        prev_btn = {i: buttons[i] for i in range(len(buttons))}

        # drive (left stick Y) + turn (right stick X): only while balancing AND in joystick mode.
        # In computer mode the controller stays silent so MQTT/autonomy drive commands aren't stomped.
        if state["en"] and drive_mode == "joystick":
            # VELOCITY drive: the stick commands a SPEED (m/s), not an accumulating position.
            # Firmware drives at this speed while non-zero and latches the current position as
            # the hold-home the instant it returns to 0 (-> drives smooth, stops where you let
            # go, no windup/overshoot). 'ptgt' position moves are still honored separately.
            d = dz(-axes[AXIS_DRIVE]) if AXIS_DRIVE < len(axes) else 0.0   # stick up = forward
            vel = d * MAX_SPEED
            # send on meaningful change (incl. return to 0) OR as a periodic heartbeat
            if ((last_vel is None or abs(vel - last_vel) > 0.01) or
                    (now - last_send >= 0.5)) and now - last_send >= send_period:
                last_send = now
                last_vel = vel
                send("dv %.3f" % vel)
            # turn: right stick X -> commanded yaw rate (rad/s); send on change incl. return to 0
            tn = dz(axes[AXIS_TURN]) if AXIS_TURN < len(axes) else 0.0
            yaw_cmd = TURN_SIGN * tn * MAX_YAW_RATE
            if (last_turn is None or abs(yaw_cmd - last_turn) > 0.05) and now - last_turn_send >= send_period:
                last_turn_send = now
                last_turn = yaw_cmd
                send("turnrate %.2f" % yaw_cmd)
        else:
            last_vel = None
            last_turn = None

        time.sleep(period)

    # clean exit (leave the robot's balance state alone -- the bridge keeps it running)
    try:
        mqc.loop_stop(); mqc.disconnect()
    except Exception:
        pass


if __name__ == "__main__":
    main()
