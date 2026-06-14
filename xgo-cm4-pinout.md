# XGO Rider (CM4) — pin & button reference

Sources: `Rider-Pi_SCH.pdf` (ESP32 mainboard schematic, in this folder) + the XGO CM4
software stack (`key.py`, the WS2812 LED driver) — original at `~/Downloads/RaspberryPi-CM4-main`.

## LCD HAT buttons — Raspberry Pi GPIO (BCM), active-low, internal pull-up

Mapped **by physical position** on 2026-06-14 (full press test, all four confirmed):

| Position     | GPIO | XGO label | Use in this project                         |
|--------------|------|-----------|---------------------------------------------|
| upper-left   | 17   | C (key3)  | balance start/stop toggle (next to "RIDER") |
| upper-right  | 22   | D (key4)  | (free)                                      |
| lower-left   | 23   | B (key2)  | **hold ~1.5 s → `sudo poweroff`**           |
| lower-right  | 24   | A (key1)  | (free)                                      |

Read on the **Pi** via lgpio in `rider_status_screen.py` (NOT on the ESP32). GPIO label
order from XGO `key.py`: key1=A=24, key2=B=23, key3=C=17, key4=D=22.

## ESP32 mainboard LEDs (Rider-Pi_SCH.pdf — ESP32 GPIO, driven by firmware)

- **4× WS2812B addressable RGB**, daisy-chained on **ESP32 IO27** (5 V): IO27 → RGB1 → RGB2 → RGB3 → RGB4.
  (These are the "4 expansion-board LEDs.")
- Single status LEDs: **red = IO22**, **blue = IO23** (via 3.3k).
- NOTE: ESP32 GPIO namespace — distinct from the Pi GPIO of the buttons above.

## Other ESP32 mainboard pins (from the schematic)

- Servo bus, 1 Mbps: **IO13 (RX2) / IO14 (TX2)**.
- Pi UART link: **IO4 (RX1) / IO5 (TX1)**.
- IMU ICM-42670 over I2C: **SDA = IO18, SCL = IO19**. Battery sense divider: **IO33**.
