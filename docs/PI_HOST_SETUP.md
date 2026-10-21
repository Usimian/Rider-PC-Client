# Rider Pi — host setup (rebuild from bare metal)

Everything on the Pi's SD card that is **not** in a stock image and **not** installed by
`pi/deploy_bridge.sh`. This is the doc that makes the burn-down test pass: with this file, the
repo, and new hardware, the Pi is reconstructible without any external memory.

`deploy_bridge.sh` installs the *programs, services, and their pip deps*. This file covers the
*host state underneath them*: kernel, WiFi, kernel modules, groups, venvs, assets, controller
pairing. Do this first, then run the deploy.

## Required secrets (supply these — not in the repo)

This repo is **public**, so no secret values are committed. Exactly one secret is needed to
rebuild, and this is the whole list:

| Secret | What it's for | Where used |
|---|---|---|
| `NICOU_WIFI_PSK` | the `nicou` 5 GHz WiFi passphrase (WPA2-PSK) | step 5 |
| `PHONE_HOTSPOT_SSID`, `PHONE_HOTSPOT_PSK` | Marc's iPhone hotspot name + password (the field network) | step 5b |

Set it up once, at the start:

```bash
cp pi/secrets.env.example pi/secrets.env   # pi/secrets.env is gitignored
$EDITOR pi/secrets.env                      # fill in every value
set -a; . pi/secrets.env; set +a            # export it for the steps below
```

If `NICOU_WIFI_PSK` is set, nothing else in this runbook needs a value you'd have to remember.

## Baseline

*(Corrected 2026-10-19 from an inventory of the running card — an earlier version of this section
said "stock XGO image"; the card was in fact plain Raspberry Pi OS.)*

Flash **Raspberry Pi OS Lite, 64-bit (trixie)** with Raspberry Pi Imager — the running card is the
`2025-12-04` reference release (`/etc/rpi-issue`), any later trixie Lite is fine. In Imager's
OS-customisation:

- hostname **`raspberrypi`**, user **`pi`** (the services and paths assume `/home/pi`)
- SSH **on, public-key only**, pasting the workstation's key (`~/.ssh/id_ed25519.pub`, comment
  `marc@marc-6000`)
- WiFi: leave it to step 5, but set **wireless country `US`** (it lands on the kernel cmdline as
  `cfg80211.ieee80211_regdom=US`)

After first boot: the alias `rider` keeps working if the router hands the same address to the
same WiFi MAC; the host key changes, so on the workstation run `ssh-keygen -R <the rider address>`
once. Find the address with `ssh -G rider | grep -i '^hostname'` — never hardcode it in source.

Two things the project needs that are **not** on PyPI or apt:

- **`xgoscreen`** — the 2-inch LCD driver (`LCD_2inch.py`, an ST7789 SPI driver, + two stub files).
  **It is not on PyPI and not in either vendor tree.** Copies: the original card, and
  `vendor/xgoscreen/` in this repo's working tree (pulled off the card 2026-10-19; gitignored
  because its license is unknown and this repo is public). Installed in step 7.
- **`/home/pi/model/msyh.ttc`** — the font the LCD renders with. It is **just a symlink** to the
  apt font `fonts-wqy-microhei` (step 9), not a file from any image.

Hardware target: **Compute Module 5** on an `XGO-CM4-V1.1` carrier. Reach it as `ssh rider`
(set the alias's `HostName` once; never hardcode the IP). Full hardware map: [`HARDWARE.md`](HARDWARE.md).

## 0. apt packages

Everything the services and venvs need from Debian (from the card's apt history, filtered to what
`pi/*.py` actually imports; `smbus2`/`vl53l5cx-ctypes`/`pygame` come from pip in step 7):

```bash
sudo apt-get update
sudo apt-get install -y git python3-pip python3-venv python3-dev build-essential \
  i2c-tools python3-smbus swig python3-lgpio liblgpio-dev \
  mosquitto mosquitto-clients python3-paho-mqtt \
  python3-picamera2 python3-opencv python3-pygame \
  fonts-wqy-microhei
```

`python3-picamera2` + `python3-opencv` must be the **apt** builds (libcamera bindings); `xgovenv`
sees them through `--system-site-packages`. (The card also had `pyaudio`/`espeak-ng`/`vosk` from
an earlier voice experiment — nothing in `pi/` uses them; leave them out.)

## 1. Kernel

Running **`6.18.39+rpt-rpi-2712`**, no apt holds. If the flashed image is older, upgrade
(`sudo apt-get install linux-image-rpi-2712`). The CVE history behind this exact version — and
the `algif_aead` block installed in step 4 — is in [`SECURITY_COPYFAIL.md`](SECURITY_COPYFAIL.md).
Do **not** re-hold the kernel: the old DS4 hold was an ERTM problem, not a kernel one (see step 8).

## 2. Boot config: serial, buses, audio, camera

`rider-bridge` owns `/dev/ttyAMA0` as the single link to the ESP32, so the serial **console**
must be off: no `console=ttyAMA0,...` in `/boot/firmware/cmdline.txt`, and
`serial-getty@ttyAMA0` disabled. `sudo raspi-config` → Interface → Serial → login shell **no**,
hardware **yes**. Verify: `grep ttyAMA0 /boot/firmware/cmdline.txt` returns nothing.

The hardware interfaces live in **`/boot/firmware/config.txt`**. These are the lines on the
running card that differ from the stock Lite image (without them the LCD, ToF sensor, WM8960
beeper and camera are all dead). Add/ensure, in the top `[all]` section:

```ini
dtparam=i2c_arm=on
dtparam=spi=on
dtparam=audio=on
enable_uart=1
gpu_mem=128
dtoverlay=i2s-mmap
dtoverlay=wm8960-soundcard
camera_auto_detect=0
dtoverlay=ov5647,cam0
```

and under `[cm5]` keep `otg_mode=1` (stock) and add **`arm_freq=2000`** — a 2.0 GHz CPU cap (stock
2.4 GHz). The mainboard's 5 V path sags to ~4.7 V at the CM5 under full CPU load and trips its
under-voltage throttle; the cap trims the current peaks. The robot's services use ~10-15 % CPU.
(Measurements: `HARDWARE.md` → Power.) Reboot, then verify: `ls /dev/i2c-1 /dev/spidev0.0`,
`aplay -l | grep wm8960`, `rpicam-hello --list-cameras` or `python3 -c "from picamera2 import
Picamera2; print(Picamera2.global_camera_info())"` shows the ov5647.

## 3. Group memberships (user `pi`)

Needs, beyond the stock set: `dialout` (serial), `gpio` (buttons/LEDs), `i2c` (ToF + audio
codec), `spi` (LCD), `input` (DS4 js0), `audio` (beep).

```bash
sudo usermod -aG dialout,gpio,i2c,spi,input,audio pi   # log out/in to take effect
```

## 4. Kernel modules + modprobe drop-ins

```bash
# load at boot
printf 'joydev\nhid_sony\nhid_playstation\n' | sudo tee /etc/modules-load.d/gamepad.conf   # DS4 (hid_playstation is the driver that works; hid_sony does not)
echo i2c-dev | sudo tee /etc/modules-load.d/i2c-dev.conf                                    # /dev/i2c-1 for ToF + WM8960

# modprobe policy
printf '# CVE-2026-31431 (copy.fail) belt-and-braces -- see SECURITY_COPYFAIL.md\ninstall algif_aead /bin/false\n' | sudo tee /etc/modprobe.d/disable-algif_aead.conf
echo 'options rfkill default_state=0' | sudo tee /etc/modprobe.d/rfkill_default.conf        # radios un-blocked at boot
echo 'blacklist 8192cu' | sudo tee /etc/modprobe.d/blacklist-8192cu.conf                    # keep a stray USB-wifi dongle driver out of the way
```

> Do **not** add `/etc/modprobe.d/ds4-bluetooth.conf` with `disable_ertm=1`. That is the
> universally-cited "DS4 on Linux" fix, and on this Pi it is **wrong** — it breaks `js0`. ERTM
> must stay at its default (enabled). Its absence is deliberate.

## 5. WiFi (`nicou`, 5 GHz)

```bash
sudo nmcli connection add type wifi con-name nicou ifname wlan0 ssid nicou
sudo nmcli connection modify nicou \
  802-11-wireless.band a \
  802-11-wireless-security.key-mgmt wpa-psk 802-11-wireless-security.proto rsn \
  connection.autoconnect yes
sudo nmcli connection modify nicou wifi-sec.psk "$NICOU_WIFI_PSK"   # from pi/secrets.env (see Required secrets)
# powersave off -- powersave causes RTT spikes that mimic faults
printf '[connection]\nwifi.powersave = 2\n' | sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf
sudo systemctl restart NetworkManager
```

`band a` (5 GHz, no BSSID pin) is deliberate — the 2.4 GHz `nicou` node had WPA handshake
timeouts. **Open caveat:** 5 GHz cold-boot reliability isn't fully validated; if a headless
cold boot comes up with no WiFi, `journalctl -b -1` for `reason=15`, and the fallback is
`nmcli con mod nicou 802-11-wireless.band bg`.

### 5b. Field network: the phone's hotspot

Away from home the Pi joins Marc's iPhone Personal Hotspot; the phone's browser then opens the Rider
page at the Pi's address on that network (the LCD shows it, right of the ToF line; iPhone hotspots hand
out `172.20.10.x`, `http://raspberrypi.local` usually works too). Home wins whenever both are in range
(autoconnect priority 10 vs 5). The WiFi watchdog needs nothing: the phone is the router it pings.

```bash
sudo nmcli connection add type wifi con-name phone ifname wlan0 ssid "$PHONE_HOTSPOT_SSID"
sudo nmcli connection modify phone \
  802-11-wireless-security.key-mgmt wpa-psk wifi-sec.psk "$PHONE_HOTSPOT_PSK" \
  connection.autoconnect yes connection.autoconnect-priority 5
sudo nmcli connection modify nicou connection.autoconnect-priority 10
```

No band pin: with "Maximize Compatibility" on, the iPhone offers 2.4 GHz only. Turn the hotspot on
(and keep its settings screen open) before powering the Rider -- iOS stops advertising it when idle.

## 6. Persistent logging

Stock RPi OS forces journald to volatile (RAM), so failed-boot logs vanish. Override:

```bash
printf '[Journal]\nStorage=persistent\nSystemMaxUse=100M\n' | sudo tee /etc/systemd/journald.conf.d/99-persistent.conf
sudo systemctl restart systemd-journald
```

## 7. Python venvs

Two venvs. `deploy_bridge.sh` installs *most* deps, but assumes the venvs exist and that a few
heavy/vendored packages are already present. Create them and pre-seed those:

```bash
# xgovenv -- bridge, joystick, camera, tof-safety. xgoscreen is NOT on PyPI (see Baseline).
python3 -m venv --system-site-packages /home/pi/xgovenv
/home/pi/xgovenv/bin/pip install paho-mqtt lgpio rpi-lgpio pyserial psutil pillow pygame spidev
#   pygame = the DS4 joystick reader; spidev = the LCD driver's SPI; the rest deploy also installs.
# the LCD driver, from the workstation's repo working tree:
#   scp -r vendor/xgoscreen rider:/home/pi/xgovenv/lib/python3.13/site-packages/
#   (adjust python3.13 to the image's Python; check: ssh rider '/home/pi/xgovenv/bin/python -c "import xgoscreen.LCD_2inch"')

# tofvenv -- rider_tof.py only. vl53l5cx-ctypes builds ST's ULD + bundles ~84KB sensor fw.
python3 -m venv /home/pi/tofvenv
/home/pi/tofvenv/bin/pip install vl53l5cx-ctypes smbus2 paho-mqtt
```

Headless pygame needs `SDL_VIDEODRIVER=dummy` (already set in the joystick unit).

## 8. DS4 controller pairing

Genuine DualShock 4 (VID `054C:09CC`, MAC `98:B6:E9:29:18:54`). ERTM at **default** (step 4). To pair:

```bash
# clear any stale bond on the Pi AND on the workstation (it will steal the pad otherwise)
bluetoothctl remove <mac>; sudo rm -rf /var/lib/bluetooth/*/<mac>
# reset the pad (paperclip in the back pinhole); hold Share+PS until the bar double-flashes
bluetoothctl --  # then: power on / agent on / scan on  (keep scan ON through pair) / pair <mac> / trust <mac>
```

Verify: `/dev/input/js0` exists and `journalctl -u rider-joystick` shows
`joystick: Wireless Controller axes 6 buttons 13`. Bonded+trusted → auto-reconnects on PS press.

## 9. Runtime assets

- **`/home/pi/beep.wav`** — ships from this repo (`pi/assets/beep.wav`); `deploy_bridge.sh`
  copies it. 16 kHz mono PCM; the joystick service plays it on mode changes via the WM8960.
- **`/home/pi/model/msyh.ttc`** — LCD font. A symlink to the apt font (step 0):
  `mkdir -p /home/pi/model && ln -sf /usr/share/fonts/truetype/wqy/wqy-microhei.ttc /home/pi/model/msyh.ttc`

## 10. Deploy the stack

Now run the deploy from the workstation (repo root):

```bash
pi/deploy_bridge.sh rider
```

It installs the six units (`rider-bridge`, `-joystick`, `-camera`, `-tof`, `-tof-safety`,
`-recorder`), copies `beep.wav`, and enables + starts everything. It also installs the **broker
config** `pi/mosquitto-rider.conf` → `/etc/mosquitto/conf.d/rider.conf` (1883 open to the LAN —
a stock Mosquitto 2.x is localhost-only, so without it nothing off the Pi can connect — and 9001
for the browser page) and the **web page** `pi/web/` → `/srv/rider-web/`, then restarts the
broker. Deploy with the robot disabled. Page: `http://raspberrypi.local:9001/`. Full service description:
[`BRIDGE_SETUP.md`](BRIDGE_SETUP.md).

`rider-tof-safety` runs at boot like the others, but it only limits forward speed while switched on
from the web page (retained `rider/safety/enable {"on": true}`); with nothing retained it is off.
(2026-10-18 to -19 it was instead stopped + disabled by hand because of false triggers; the switch
replaced that local deviation.)

## Card upkeep

- **TRIM.** On 2026-10-19 the card had slowed to 22 MB/s sequential / ~560 small reads per second and
  a 19 s boot; one `sudo fstrim -v /` (18.5 GiB trimmed — it had evidently never run) brought it back
  to 40 MB/s / ~1830 reads/s and an 8 s boot. The stock weekly `fstrim.timer` only fires while the Pi
  is on, which the robot rarely is for long — run `sudo fstrim -v /` by hand now and then.
- **`/home/pi/riderlog.txt`** (the always-on recorder) is never rotated; it reached 966 MB. Archive
  it to the workstation (`ssh rider 'gzip -c riderlog.txt' > data/…`, `data/` is gitignored) and
  clear it with `sudo truncate -s 0` (root-owned, the unit appends, so it keeps writing).
- **Under-voltage** is logged on every boot (5–34 events) — a 5 V supply issue on the carrier,
  unrelated to the card, but brown-outs during writes are how SD cards get corrupted.

## Verify the rebuild

```bash
ssh rider 'uname -r; for s in bridge joystick camera tof tof-safety recorder; do \
  printf "%-10s %s\n" $s "$(systemctl is-active rider-$s.service)"; done'
ssh rider 'timeout 3 mosquitto_sub -h localhost -t rider/debug/telem -C 1'   # ESP32 telemetry
ssh rider 'timeout 3 mosquitto_sub -h localhost -t rider/tof -C 1 | head -c 80'  # ToF frames
```
