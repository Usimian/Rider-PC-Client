# Rider Pi — CVE-2026-31431 ("Copy Fail")

**Status (2026-08-13): patched at source.** The Pi now runs kernel **6.18.39+rpt-rpi-2712**,
which carries the `authencesn` fix, so the CVE is closed by the kernel itself. The
`algif_aead` module-block described below is **kept as belt-and-braces**, not the primary fix.
See [The proper fix — done](#the-proper-fix--done) for how the upgrade went.

This began as a *mitigation* while the kernel was held on the vulnerable 6.12.47 (the history
below is kept because it explains the block that's still in place, and because a reimage may
land on an old kernel again). Reach the Pi with `ssh rider`. The module-block is Pi **host
state**, not code — it lives on the SD card outside the repo, so it is recorded here.
**If the Pi is ever reimaged onto a pre-6.18.33 kernel, re-apply the block** (see [Applying](#applying)).

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

## Why a mitigation was needed first (historical)

> Resolved 2026-08-13 by the kernel upgrade above; kept for context on the block that remains.

The Pi's kernel *was* **deliberately held** on 6.12.47:

```
$ apt-mark showhold
linux-image-6.12.47+rpt-rpi-2712
```

That hold was placed 2026-06-13 to keep the DualShock 4 controller working, after an
upgrade to 6.18.33 coincided with the DS4 failing to produce `/dev/input/js0`. A held
kernel receives no security updates — so the hold that protected the controller also
froze the Pi on a vulnerable kernel. (The hold has since been released and the DS4 shown
to work on 6.18.39 — see above — so this is no longer a live constraint.)

The two kernels differ exactly where it matters. From the packaged changelogs:

| Kernel | `crypto: authencesn - reject too-short AAD (assoclen<8)` |
|---|---|
| 6.12.47+rpt-rpi-2712 (the formerly-held kernel) | **absent** |
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

## The proper fix — done

Applied 2026-08-13 at the bench (robot on the stand, balance stopped, physical access):

```bash
sudo apt-get update
sudo apt-mark unhold linux-image-6.12.47+rpt-rpi-2712
sudo apt-get install linux-image-rpi-2712    # pulled 6.18.39+rpt-rpi-2712
sudo systemctl reboot
```

The install was kernel-only (0 packages removed); `6.12.47` stays installed as a rollback.

**The DS4 hypothesis was right — the hold was unnecessary.** The 2026-06-13 rollback had
changed two things at once (the kernel *and* `/etc/modprobe.d/ds4-bluetooth.conf`, which
forced `disable_ertm=1`). ERTM was the *proven* culprit; the kernel was never tested alone.
That file is now absent, so this was the first trial of **6.18 with ERTM at default** — and
the DS4 works: after reboot it paired and produced `/dev/input/js0`, and `rider-joystick`
read it (`axes 6 buttons 13`, button events flowing). The exact failure the hold guarded
against (upgrade → no `js0`) does not occur on 6.18.39.

### Post-upgrade verification (2026-08-13, all confirmed)

- WiFi returned, `uname -r` = `6.18.39+rpt-rpi-2712`, `algif_aead` still blocked + unloaded.
- All six services active/enabled; ESP32 telemetry flowing; battery/IMU live.
- ToF ranging (64/64 zones), safety governor publishing (cliff/obstacle both seen working),
  camera capture round-trip OK, I²C (WM8960 + ToF), BT controller up, audio card enumerated.
- **DS4** paired → `js0` present → joystick service reads it. Stick→wheels was **not**
  exercised: it is gated behind `en=1` (balancing) and additionally forward-capped by the
  cliff governor, so it cannot be tested on a stand — that is a floor test for later, not a
  kernel question.

**Kept the `algif_aead` block** after the upgrade — costs nothing, same belt-and-braces
posture Ubuntu ships by default, and it covers a reimage landing on an old kernel.

### Still open

- **Cold-boot WiFi reliability.** Only one *warm* reboot has been done. 5 GHz `band=a`
  (set 2026-06-18) was never validated across multiple *cold* boots — worth a few power
  cycles before trusting a headless boot.

### Rollback (if ever needed)

`6.12.47` remains installed. To revert:

```bash
sudo apt-mark hold linux-image-6.12.47+rpt-rpi-2712   # optional: pin it again
# set kernel_2712.img back to 6.12.47, or remove the 6.18 image, then:
sudo systemctl reboot
```

## Fleet status (2026-08-13)

| Machine | Kernel patched | `algif_aead` blocked |
|---|---|---|
| Workstation (Ubuntu 24.04, 7.0.0-28) | yes | yes (Ubuntu `kmod` 7.2) |
| Yahboom (Ubuntu 24.04, 6.8.12-tegra) | unconfirmed — L4T kernel, no changelog | yes, verified |
| Rider Pi (RPi OS trixie, **6.18.39** since 2026-08-13) | **yes** — kernel carries the fix | yes, kept as belt-and-braces |
| Rider ESP32 | n/a — not Linux | n/a |

The ESP32 is out of scope entirely: no kernel, no page cache, no privilege boundary to
escalate across. The rule is simply *if it boots Linux, it's in scope.*
