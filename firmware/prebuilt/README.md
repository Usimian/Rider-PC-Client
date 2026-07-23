# Prebuilt firmware images (known-good, flashable)

These `.bin`s are **tracked on purpose** (a negation in the repo `.gitignore`
overrides the repo-wide `*.bin` ignore) so they stay preserved in the repo.

| File | What it is | Flash |
|---|---|---|
| `rider_passthrough_fw.bin` | Servo-bus passthrough firmware (source: `firmware/esp32_passthrough/`) — needed for `tools/servo/*` | `cd firmware/esp32_passthrough && pio run -t upload` |
| `rider_stock_R-1.1.3_full_backup.bin` | **Original/factory** full 4 MB flash backup (stock `R-1.1.3`) | `esptool write_flash 0x0 <file>` — full-flash restore to factory |
| `rider_stock_R-1.1.3_emergency_backup_2026-04-03.bin` | Second full 4 MB dump of the same stock `R-1.1.3` (folded in from the retired `~/rider_esp32/`) | `esptool write_flash 0x0 <file>` — equivalent factory restore |
| `rider_stock_R-1.1.6_app.bin` | Stock **`R-1.1.6`** app partition only (1.1 MB) — a *newer* factory firmware than either full backup | app-slot flash, see below |

The passthrough `_fw.bin` is rebuildable from source; kept here as a known-good snapshot.
The stock backup is the factory restore image (originally at `~/Downloads/Rider-bins/`).

The two stock dumps are interchangeable as restore images: their `app0`, `app1`, and
`spiffs` partitions are byte-identical. They differ only inside `nvs` (`0x9000`–`0xB000`,
8 KB) — BT/PHY calibration blobs the ESP32 regenerates on boot, with identical BLE keys,
so both came off this same chip. Verify with:

```bash
cmp -l rider_stock_R-1.1.3_full_backup.bin rider_stock_R-1.1.3_emergency_backup_2026-04-03.bin | awk '{print int($1/4096)*4096}' | uniq
```

## Stock `R-1.1.6` (`rider_stock_R-1.1.6_app.bin`)

This is the **only copy that exists** — folded in from `~/Downloads/Rider-bins/` because
nothing else in the repo carries `R-1.1.6`. Both full backups above are `R-1.1.3`, so this
is an *upgrade* image, not a restore-to-known-state one. It has never been dumped off a
running chip here, so there is no `R-1.1.6` nvs/spiffs to pair with it.

It is the app partition alone. The other three pieces of a stock flash are already inside
`rider_stock_R-1.1.3_full_backup.bin`, byte-for-byte, at the offsets from its partition
table — extract them rather than hunting for separate files:

```bash
B=rider_stock_R-1.1.3_full_backup.bin
dd if=$B of=bootloader.bin bs=1 skip=$((0x1000))  count=$((0x4440)) status=none  # 0x1000
dd if=$B of=partitions.bin bs=1 skip=$((0x8000))  count=$((0xC00))  status=none  # 0x8000
dd if=$B of=otadata.bin    bs=1 skip=$((0xE000))  count=$((0x2000)) status=none  # 0xE000
esptool write_flash 0x1000 bootloader.bin 0x8000 partitions.bin \
                    0xe000 otadata.bin   0x10000 rider_stock_R-1.1.6_app.bin
```

Writing `otadata` is what forces the boot back to the `app0` slot you just wrote. Flashing
the app alone at `0x10000` without it can leave the ESP32 booting the stale `app1` image.
Partition layout is identical between `R-1.1.3` and `R-1.1.6`, so the offsets hold.

The self-balance firmware is **not** snapshotted here — it changes every firmware
commit, so build + flash it from source: `cd firmware/esp32_rider_fw && pio run -t upload`
(default env `esp32_lqr`, or `-e esp32` for the policy build).
