# Rider Pi — host setup (rebuild from bare metal)

Everything on the Pi's SD card that is **not** in a stock image and **not** installed by
`pi/deploy_bridge.sh`. This is the doc that makes the burn-down test pass: with this file, the
repo, and new hardware, the Pi is reconstructible without any external memory.

`deploy_bridge.sh` installs the *programs, services, and their pip deps*. This file covers the
*host state underneath them*: kernel, WiFi, kernel modules, groups, venvs, assets, controller
pairing. Do this first, then run the deploy.

## Baseline

Start from the **stock XGO Rider Raspberry Pi OS image** (RPi OS trixie, arm64) flashed to the
SD card. That image is the source of two things this project does **not** vendor:

- **`xgoscreen`** — the 2-inch LCD library (lives in `xgovenv/.../site-packages/xgoscreen`, not
  on PyPI). From the XGO / Luwu-OS distribution.
- **`/home/pi/model/msyh.ttc`** — the CJK font the LCD renders with.

Hardware target: **Compute Module 5** on an `XGO-CM4-V1.1` carrier. Reach it as `ssh rider`
(set the alias's `HostName` once; never hardcode the IP). Full hardware map: [`HARDWARE.md`](HARDWARE.md).

## 1. Kernel

Running **`6.18.39+rpt-rpi-2712`**, no apt holds. If the flashed image is older, upgrade
(`sudo apt-get install linux-image-rpi-2712`). The CVE history behind this exact version — and
the `algif_aead` block installed in step 4 — is in [`SECURITY_COPYFAIL.md`](SECURITY_COPYFAIL.md).
Do **not** re-hold the kernel: the old DS4 hold was an ERTM problem, not a kernel one (see step 8).

## 2. Serial / UART

`rider-bridge` owns `/dev/ttyAMA0` as the single link to the ESP32, so the serial **console**
must be off: no `console=ttyAMA0,...` in `/boot/firmware/cmdline.txt`, and
`serial-getty@ttyAMA0` disabled. `sudo raspi-config` → Interface → Serial → login shell **no**,
hardware **yes**. Verify: `grep ttyAMA0 /boot/firmware/cmdline.txt` returns nothing.

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
sudo nmcli connection modify nicou wifi-sec.psk '<the nicou passphrase>'
# powersave off -- powersave causes RTT spikes that mimic faults
printf '[connection]\nwifi.powersave = 2\n' | sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf
sudo systemctl restart NetworkManager
```

`band a` (5 GHz, no BSSID pin) is deliberate — the 2.4 GHz `nicou` node had WPA handshake
timeouts. **Open caveat:** 5 GHz cold-boot reliability isn't fully validated; if a headless
cold boot comes up with no WiFi, `journalctl -b -1` for `reason=15`, and the fallback is
`nmcli con mod nicou 802-11-wireless.band bg`.

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
# xgovenv -- bridge, joystick, camera, tof-safety. xgoscreen is vendored (see Baseline).
python3 -m venv --system-site-packages /home/pi/xgovenv
/home/pi/xgovenv/bin/pip install paho-mqtt lgpio rpi-lgpio pyserial psutil pillow pygame
#   pygame = the DS4 joystick reader; the rest deploy also installs (idempotent).

# tofvenv -- rider_tof.py only. vl53l5cx-ctypes builds ST's ULD + bundles ~84KB sensor fw.
python3 -m venv /home/pi/tofvenv
/home/pi/tofvenv/bin/pip install vl53l5cx-ctypes smbus2 paho-mqtt
```

Headless pygame needs `SDL_VIDEODRIVER=dummy` (already set in the joystick unit).

## 8. DS4 controller pairing

Genuine DualShock 4 (VID `054C:09CC`). ERTM at **default** (step 4). To pair:

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
- **`/home/pi/model/msyh.ttc`** — LCD font, from the stock image (Baseline).

## 10. Deploy the stack

Now run the deploy from the workstation (repo root):

```bash
pi/deploy_bridge.sh rider
```

It installs the six units (`rider-bridge`, `-joystick`, `-camera`, `-tof`, `-tof-safety`,
`-recorder`), copies `beep.wav`, and enables + starts everything. Full service description:
[`BRIDGE_SETUP.md`](BRIDGE_SETUP.md).

## Verify the rebuild

```bash
ssh rider 'uname -r; for s in bridge joystick camera tof tof-safety recorder; do \
  printf "%-10s %s\n" $s "$(systemctl is-active rider-$s.service)"; done'
ssh rider 'timeout 3 mosquitto_sub -h localhost -t rider/debug/telem -C 1'   # ESP32 telemetry
ssh rider 'timeout 3 mosquitto_sub -h localhost -t rider/tof -C 1 | head -c 80'  # ToF frames
```
