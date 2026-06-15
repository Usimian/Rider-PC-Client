# Prebuilt firmware images (known-good, flashable)

These three `.bin`s are **tracked on purpose** (a negation in the repo `.gitignore`
overrides the repo-wide `*.bin` ignore) so they stay preserved in the repo.

| File | What it is | Flash |
|---|---|---|
| `rider_balance_fw.bin` | Current self-balance firmware (source: `firmware/esp32_rider_fw/`) | `cd firmware/esp32_rider_fw && pio run -t upload` (rebuild), or esptool app @ `0x10000` |
| `rider_passthrough_fw.bin` | Servo-bus passthrough firmware (source: `firmware/esp32_passthrough/`) — needed for `tools/servo/*` | `cd firmware/esp32_passthrough && pio run -t upload` |
| `rider_stock_R-1.1.3_full_backup.bin` | **Original/factory** full 4 MB flash backup (stock `R-1.1.3`) | `esptool write_flash 0x0 <file>` — full-flash restore to factory |

The two `_fw.bin` are rebuildable from source; kept here as known-good snapshots.
The stock backup is the factory restore image (originally at `~/Downloads/Rider-bins/`).
