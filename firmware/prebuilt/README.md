# Prebuilt firmware images (known-good, flashable)

These `.bin`s are **tracked on purpose** (a negation in the repo `.gitignore`
overrides the repo-wide `*.bin` ignore) so they stay preserved in the repo.

| File | What it is | Flash |
|---|---|---|
| `rider_passthrough_fw.bin` | Servo-bus passthrough firmware (source: `firmware/esp32_passthrough/`) — needed for `tools/servo/*` | `cd firmware/esp32_passthrough && pio run -t upload` |
| `rider_stock_R-1.1.3_full_backup.bin` | **Original/factory** full 4 MB flash backup (stock `R-1.1.3`) | `esptool write_flash 0x0 <file>` — full-flash restore to factory |

The passthrough `_fw.bin` is rebuildable from source; kept here as a known-good snapshot.
The stock backup is the factory restore image (originally at `~/Downloads/Rider-bins/`).

The self-balance firmware is **not** snapshotted here — it changes every firmware
commit, so build + flash it from source: `cd firmware/esp32_rider_fw && pio run -t upload`
(default env `esp32_lqr`, or `-e esp32` for the policy build).
