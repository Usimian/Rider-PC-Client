# Prebuilt firmware images (known-good, flashable)

These `.bin`s are **tracked on purpose** (a negation in the repo `.gitignore`
overrides the repo-wide `*.bin` ignore) so they stay preserved in the repo.

| File | What it is | Flash |
|---|---|---|
| `rider_passthrough_fw.bin` | Servo-bus passthrough firmware (source: `firmware/esp32_passthrough/`) — needed for `tools/servo/*` | `cd firmware/esp32_passthrough && pio run -t upload` |
| `rider_stock_R-1.1.3_full_backup.bin` | **Original/factory** full 4 MB flash backup (stock `R-1.1.3`) | `esptool write_flash 0x0 <file>` — full-flash restore to factory |
| `rider_stock_R-1.1.3_emergency_backup_2026-04-03.bin` | Second full 4 MB dump of the same stock `R-1.1.3` (folded in from the retired `~/rider_esp32/`) | `esptool write_flash 0x0 <file>` — equivalent factory restore |

The passthrough `_fw.bin` is rebuildable from source; kept here as a known-good snapshot.
The stock backup is the factory restore image (originally at `~/Downloads/Rider-bins/`).

The two stock dumps are interchangeable as restore images: their `app0`, `app1`, and
`spiffs` partitions are byte-identical. They differ only inside `nvs` (`0x9000`–`0xB000`,
8 KB) — BT/PHY calibration blobs the ESP32 regenerates on boot, with identical BLE keys,
so both came off this same chip. Verify with:

```bash
cmp -l rider_stock_R-1.1.3_full_backup.bin rider_stock_R-1.1.3_emergency_backup_2026-04-03.bin | awk '{print int($1/4096)*4096}' | uniq
```

The self-balance firmware is **not** snapshotted here — it changes every firmware
commit, so build + flash it from source: `cd firmware/esp32_rider_fw && pio run -t upload`
(default env `esp32_lqr`, or `-e esp32` for the policy build).
