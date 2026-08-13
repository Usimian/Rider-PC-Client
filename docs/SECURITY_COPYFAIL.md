# Rider Pi — CVE-2026-31431 ("Copy Fail") mitigation

Applied 2026-08-13 to the Rider's Raspberry Pi (reach it with `ssh rider`). This is Pi **host
state**, not code — it lives outside the repo on the SD card, so it is recorded here.
**If the Pi is ever reimaged, re-apply it** (see [Applying](#applying)).

## What the vulnerability is

CVE-2026-31431, disclosed to oss-security 2026-04-29. A logic bug in the kernel's
`authencesn` crypto template: it fails to reject too-short associated data
(`assoclen < 8`). Reached through an `AF_ALG` socket on the `algif_aead` interface and
chained through `splice()`, an **unprivileged local user can write 4 controlled bytes
into the page cache of any file they can read** — point that at a setuid binary's
cached pages and you have root.

It is deterministic: no race, no heap grooming, no KASLR leak. The public PoC is 732
bytes of Python and works unmodified across distributions. Present in kernels shipped
since 2017.

Attack surface is **local only**. On the Rider that means someone who already has a
shell on the Pi — the SSH account or the MQTT-facing services. Lower risk than a
multi-tenant host, but not zero, and the fix is free.

## Why the Rider needs a mitigation rather than the patch

The Pi's kernel is **deliberately held**:

```
$ apt-mark showhold
linux-image-6.12.47+rpt-rpi-2712
```

That hold was placed 2026-06-13 to keep the DualShock 4 controller working, after an
upgrade to 6.18.33 coincided with the DS4 failing to produce `/dev/input/js0`. A held
kernel receives no security updates — so the hold that protects the controller also
froze the Pi on a vulnerable kernel.

The two kernels differ exactly where it matters. From the packaged changelogs:

| Kernel | `crypto: authencesn - reject too-short AAD (assoclen<8)` |
|---|---|
| 6.12.47+rpt-rpi-2712 (held, running) | **absent** |
| 6.18.33+rpt-rpi-v8 | **present** |

Both carry three older, unrelated `algif_aead` fixes (`MAY_BACKLOG`, `ctx->more`
wakeups, `ctx->init`) — those are not this CVE. Do not mistake them for it.

Raspberry Pi OS / Debian also **did not** ship the module blacklist that Ubuntu
bundled into its `kmod` package, so the Pi had no mitigation from the distro either.

## What was applied

`/etc/modprobe.d/disable-algif_aead.conf` on the Pi:

```
# Disable algif_aead due to CVE-2026-31431 (copy.fail)
# Kernel 6.12.47 is held for the DS4 controller and lacks the authencesn fix.
# Remove this once running a kernel >= 6.18.x that carries the fix.
# Applied 2026-08-13.
install algif_aead /bin/false
```

`install <mod> /bin/false` is used rather than a plain `blacklist` line on purpose:
`blacklist` only suppresses *automatic* loading, while `install ... /bin/false` also
defeats an explicit `modprobe` — which is what an attacker would do. This is the same
form Ubuntu shipped on the workstation and the Yahboom.

`algif_aead` is the only path to the bug, so blocking the module closes it on the
running kernel.

### Blast radius: none

Nothing in the Rider stack uses AEAD through `AF_ALG`. Before and after the change,
only `algif_hash` and `algif_skcipher` are loaded (`af_alg` refcount 6) — `algif_aead`
had never loaded. The bridge, joystick, camera, ToF, and recorder services are
unaffected, and no reboot is required.

## Applying

```bash
ssh rider
sudo tee /etc/modprobe.d/disable-algif_aead.conf >/dev/null <<'EOF'
# Disable algif_aead due to CVE-2026-31431 (copy.fail)
# Kernel 6.12.47 is held for the DS4 controller and lacks the authencesn fix.
# Remove this once running a kernel >= 6.18.x that carries the fix.
install algif_aead /bin/false
EOF
```

Takes effect immediately. No reboot, no service restart.

## Verifying

Do not trust the file's presence — test that the load is actually refused:

```bash
sudo modprobe algif_aead ; lsmod | grep algif_aead
```

Expected:

```
modprobe: ERROR: Error running install command '/bin/false' for module algif_aead: retcode 1
modprobe: ERROR: could not insert 'algif_aead': Invalid argument
```

and no `algif_aead` line from `lsmod`. Verified in this form on 2026-08-13.

## Retiring it — the proper fix

The mitigation closes *this* CVE. It does not patch the kernel, and the held kernel
misses every future fix. The real fix is to retire the hold:

```bash
sudo apt update
sudo apt-mark unhold linux-image-6.12.47+rpt-rpi-2712
sudo apt install linux-image-rpi-2712        # 6.18.34 at time of writing
sudo reboot
```

Then verify in order: WiFi came back → `uname -r` shows 6.18.x → press PS on the DS4
and check `/dev/input/js0` → `rider-joystick.service` active.

**The hold may be unnecessary.** The 2026-06-13 rollback changed two things at once —
the kernel *and* `/etc/modprobe.d/ds4-bluetooth.conf` (which forced `disable_ertm=1`).
The ERTM setting was the *proven* culprit; the kernel was never tested independently.
That file is now absent, so **6.18 with ERTM at default has never been tried** — the
combination most likely to work. Worth an hour at the bench.

Rollback if the DS4 fails: 6.12.47 remains installed, so re-hold it and reboot.

**Keep this blacklist even after moving to 6.18.** It costs nothing and is the same
belt-and-braces posture Ubuntu ships by default.

### Two cautions for that session

- **Rebooting this Pi is not free.** 5 GHz boot reliability (`band=a`, set 2026-06-18)
  was never validated across multiple cold boots. If WiFi doesn't come up you lose
  SSH — do it with physical access to the robot.
- **Stop the balance loop first.** The ESP32 balances independently at ~253 Hz. If the
  Pi disappears mid-reboot while the robot is actively balancing, the ESP32 keeps
  running with a dead command source. Robot on the stand, balance stopped.

## Fleet status (2026-08-13)

| Machine | Kernel patched | `algif_aead` blocked |
|---|---|---|
| Workstation (Ubuntu 24.04, 7.0.0-28) | yes | yes (Ubuntu `kmod` 7.2) |
| Yahboom (Ubuntu 24.04, 6.8.12-tegra) | unconfirmed — L4T kernel, no changelog | yes, verified |
| Rider Pi (RPi OS trixie, 6.12.47 held) | **no** | yes, verified |
| Rider ESP32 | n/a — not Linux | n/a |

The ESP32 is out of scope entirely: no kernel, no page cache, no privilege boundary to
escalate across. The rule is simply *if it boots Linux, it's in scope.*
