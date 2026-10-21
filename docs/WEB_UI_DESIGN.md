# Web control page for the Rider — design (2026-10-19, not built yet)

**Goal:** replace the Tk desktop GUI (`gui/`) with a page served by the Rider itself, so any
browser on the network — workstation, phone, tablet — can watch and control the robot with
nothing installed. Later the same page carries the Hover's controls once it joins the Rider's
bus (see the pairing plan in the Hover project).

**Non-goals for v1:** driving from the browser (the DS4 drives), a login, internet access.

## Architecture

```
 browser (any device on the LAN)
   │  HTTP  GET /            -> the page (static HTML/JS/CSS)
   │  MQTT over WebSockets   -> same messages the DS4 / Tk GUI use today
   ▼
 Rider Pi: Mosquitto 2.0.21  (built with libwebsockets — CONFIRMED on the Pi 2026-10-19)
   listener 1883  (existing: Pi services, workstation tools, later the Hover)
   listener 9001 protocol websockets + http_dir /home/pi/web   (NEW: page + browser MQTT)
   ▼
 rider-bridge (serial owner) ⇄ ESP32 ; rider-camera ; rider-tof ; rider-joystick
```

- **No new server program.** Mosquitto's WebSocket listener can also serve a folder of static
  files (`http_dir`). One config file on the Pi = page + live connection. *To verify in step 1:*
  if `http_dir` misbehaves on this build, fall back to a 10-line `python3 -m http.server`
  systemd unit serving the same folder.
- **The page is just another MQTT client.** It subscribes to what the robot already publishes and
  publishes the same command lines as the DS4 and the Tk GUI. All robot logic stays where it is.
- **Languages:** the page is plain HTML + JavaScript (the only thing a browser runs; no build
  step, no framework). The MQTT library is **MQTT.js** (MIT licence), a single `mqtt.min.js`
  **committed in the repo and served from the Pi** — nothing is loaded from the internet (Marc's
  everything-stays-local rule). Pi side: config only.
- **Repo layout:** `pi/web/` (index.html, app.js, style.css, vendor/mqtt.min.js),
  `pi/mosquitto-rider.conf` (both listeners). `pi/deploy_bridge.sh` copies both.
- **Reconstruction gap this also closes:** today the broker's `listener 1883` /
  `allow_anonymous true` lives only in `/etc/mosquitto/conf.d/` on the card — not in the repo or
  `PI_HOST_SETUP.md`. A fresh Mosquitto 2.x is localhost-only, so the workstation (and the
  Hover) could not connect after a rebuild. The new conf file in the repo fixes that.

## What the robot already publishes (CONFIRMED live 2026-10-19)

| topic | rate | content the page uses |
|---|---|---|
| `rider/status` | ~3 Hz | `roll_balance_enabled` (= **balancing on**), `roll_leveling_enabled`, `mode` (POL/LQR), `battery_level`, `battery_voltage`, `cpu_percent`, `cpu_temp`, `power_w`, `loop_hz`, `read_fails` (failed wheel reads since ESP32 boot), `fault`, `controller_connected` (DS4), `controller_battery` (DS4 %, null when not connected), `controller_charging`, `position`, `target`, `pose_x/y` |
| `rider/status/imu` | ~3 Hz | `roll`, `pitch`, `yaw` |
| `rider/status/odom` | ~3 Hz | `x`, `y`, `heading` |
| `rider/control/drivemode` | retained | `{"mode": "joystick"|"computer"}` (DS4 Options toggles it) |
| `rider/tof` | ~5 Hz | 8×8 distances `d[64]` (mm) + status `s[64]` |
| `rider/safety/fwd_limit` | per ToF frame, retained | governor: `factor` (applied), `on`, `would`, `reason`, `min_mm`, `cls` (64 zone classes) |
| `rider/debug/telem` | ~11 Hz | the raw ESP32 line (pitch, wheel speeds, loop timing…) — for a debug panel only |
| `rider/response/image_capture` | on request | `{"success", "image_data": base64 JPEG 640×480 ~20 KB}` |

## What the page sends

| control | message | notes |
|---|---|---|
| **Balance on/off** | `rider/control/line` `en 1` / `en 0` | toggles by the live `roll_balance_enabled` |
| **Roll Balance** (leveling) | `level 1` / `level 0` | by live `roll_leveling_enabled` (as the fixed Tk GUI) |
| **Controller** POLICY / LQR | `ctrl 0` / `ctrl 1` | greyed out while balancing; the firmware refuses it anyway |
| **Obstacle & edge stop** | `rider/safety/enable` `{"on": bool}`, **retained** | toggles by the governor's reported `on`; off = it reports but never limits |
| **Height ▲ ▼** (hold) | `legrate ±50` every 100 ms while held, `legrate 0` on release | the firmware stops the legs if the 100 ms heartbeat stops (300 ms timeout) — a closed tab can't leave legs moving |
| **Ride height** | `leghome` | same as DS4 R3 |
| **Top speed** slider | `posvmax v` + `posvmaxb v` (m/s) | caps full stick for both controllers; replaces the dead Tk speed slider |
| **Reset position** | `poszero` | same as DS4 Circle |
| **E-STOP** | `rider/control/line` `en 0` then `leglimp` | always visible, top bar; **same as the LCD stop button** (balance off + legs limp), not the Tk GUI's `emergency_stop` (balance off only). Built in step 2 |
| **Power off / Reboot Pi** | `rider/control/system` `poweroff_pi` / `reboot_pi` | behind a confirm dialog; the LCD shows SHUTTING DOWN / REBOOTING |
| **Camera** | `rider/control/image_capture` `{"request_id", "resolution"}` | see Video |

**Dropped from the Tk GUI:** LED brightness (removed 2026-10-19), Volume (dead — the Pi only logs
it), Body tilt (conflicts with leveling; see open questions), config save/load (nothing left to
save once the page is the client), robot-IP dialog (the page is served *by* the robot).

## Layout

```
┌───────────────────────────────────────────────────────────────────────────┐
│ RIDER  ● link  🔋 75 % 7.82 V   DS4 ●   CTRL: LQR   DRIVE: JOYSTICK   [ E-STOP ] │
├──────────────────────────────┬────────────────────────────────────────────┤
│  CAMERA  (640×480)           │  BALANCE                                    │
│                              │   [ Balance: ON ]   [ Roll Balance: OFF ]   │
│                              │   Controller: ( POLICY | LQR )  (off only)  │
│                              │   Top speed  ──●──── 0.25 m/s               │
│                              │  LEGS                                       │
│                              │   [ ▲ hold ]  [ ride height ]  [ ▼ hold ]   │
│                              │  POSE  pitch 3.5°  roll −1.8°  heading −20° │
│                              │   position −0.54 m  target −0.53 m [reset]  │
├──────────────────────────────┴────────────────────────────────────────────┤
│ ToF 8×8 heat-map │ CPU 15 % 61 °C 2.0 W │ loop 201 Hz │ read fails 0 % │ [Power off] [Reboot] │
└───────────────────────────────────────────────────────────────────────────┘
```

Narrow screens (phone) stack the columns: status bar, camera, balance, legs, details.

## Behaviour rules

- **The page holds no robot state.** Every indicator comes from the robot's own messages, so
  any number of browsers can be open at once and all agree. Buttons decide their direction from
  the live state (as the fixed Roll Balance toggle does).
- **Closing or losing the page never sends anything.** (The Tk GUI used to send an emergency stop
  on close — removed 2026-10-19 after it switched balancing off.)
- **Link indicator** turns red if no `rider/status` arrives for 2 s; controls grey out until it
  returns. MQTT.js reconnects by itself.
- **Several open pages may all send commands.** v1 accepts that (same as today: DS4 + GUI +
  tools all publish). If it becomes a problem, add a "this page has control" claim later.

## Video

- **v1:** the page requests a frame, shows it, requests the next — paced to ~3-5 frames/s
  (one 20 KB JPEG per request; the camera service already does exactly this for the Tk GUI).
- **v2 (if v1 feels choppy):** an MJPEG stream from `rider-camera` on its own HTTP port
  (`<img src=…>`), ~15 frames/s, no MQTT in the video path.
- Colour: the camera is a NoIR module (`HARDWARE.md`); dark areas stay purple until it is
  swapped for one with an IR-cut filter.

## Steps (each ends with a check on the real robot)

1. **Broker:** `pi/mosquitto-rider.conf` with both listeners + `http_dir`; deploy; a placeholder
   page loads in a browser and shows live `rider/status` JSON. Add the conf to `PI_HOST_SETUP.md`.
2. **Status bar + pose + E-stop** (read-only except E-stop).
3. **Balance card:** Balance, Roll Balance, Controller, Top speed, Reset position.
4. **Legs:** hold-to-move height + ride height (verify the heartbeat stop by closing the tab mid-move).
5. **Camera v1**, then ToF heat-map and the system buttons. (ToF card + obstacle/edge stop switch
   done first, 2026-10-19.)
6. **Retire the Tk GUI** once Marc has used the page for a while: delete `gui/` and
   `start_gui.sh`, purge references (no dead code).

## Optional later: field mode (the Rider makes its own network)

**Use case (Marc, 2026-10-19): places with no WiFi at all.** So it is a *fallback*, not a switch:

- **At boot the Pi tries the home network (`nicou`) for ~30 s. If it doesn't connect, it starts a
  hotspot** on `wlan0` (own network name + password, NetworkManager's built-in DHCP). At home
  nothing changes, so there is no way to lock ourselves out; away, the Rider brings its own
  network. A small boot-time unit does the check (NetworkManager has no "fall back to hotspot"
  setting of its own).
- **Who joins it:** a phone or laptop (for this web page) and the Hover. Mosquitto already listens
  on all interfaces, so both find the broker at the hotspot's fixed address (NetworkManager's
  shared mode gives the Pi `10.42.0.1`). The page and broker work exactly as at home.
- **Manual control (Marc, 2026-10-19):** the LCD's **upper-right button toggles the hotspot** on/off
  (freed by merging balance + stop onto the upper-left button). The Pi always boots on the home
  network, so a hotspot left on can't strand it; whether to keep the automatic boot fallback as
  well is open.
- **The Hover** keeps two saved networks: home first, the Rider's hotspot second — and must know
  the broker address for each (Hover project change).
- **Workstation:** not reachable in field mode (it's at home anyway). The hotspot password is a
  secret: it goes in the gitignored `pi/secrets.env` next to the home WiFi key.
- **One radio is enough** for this use case (no home network to stay on). The LCD shows which mode
  it came up in and the hotspot name, so a phone user knows what to join.

## Open questions for Marc

1. Drive from the browser at all (on-screen joystick, only in "computer" drive mode, with a
   heartbeat)? Out of v1 for now.
2. Body tilt: drop it, or make it the leveling target angle (`levset`)?
3. Keep a debug panel (raw telemetry, live firmware knobs like `lqrvref`) on the page, or keep
   that in workstation tools?
