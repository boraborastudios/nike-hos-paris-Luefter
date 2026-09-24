# 8-Relay Chase — Waveshare ESP32-S3-ETH-8DI-8RO

Board: [Waveshare ESP32-S3-POE-ETH-8DI-8RO-C on Amazon](https://www.amazon.de/-/en/Waveshare-ESP32-S3-POE-ETH-8DI-8RO-C-Industrial-Interface-Protection/dp/B0F93SYY5G)

Relays 1→8 switch **ON** one after another, 2 seconds apart. Then relays 1→8
switch **OFF** one after another, 2 seconds apart. Repeats forever.

## Build & upload

```bash
pio run -t upload
pio device monitor
```

First flash on this board (native USB, no CH340/CP2102 chip) sometimes needs
manual bootloader mode:
1. Hold **BOOT**, press and release **RESET**, then release **BOOT**.
2. Then run `pio run -t upload`.

(This is Waveshare's own workaround, from their FAQ, for when the port isn't
found or flashing fails.)

## How the relays are wired up

This board does **not** put the 8 relays on plain ESP32 GPIOs. They're driven
by an onboard **TCA9554PWR I2C GPIO expander** (channels EXIO1–EXIO8 = relay
1–8), sharing the I2C bus with the onboard RTC chip:

| Signal | Pin |
|---|---|
| I2C SDA | GPIO42 |
| I2C SCL | GPIO41 |
| TCA9554 address | `0x20` |

`src/main.cpp` talks to it directly over `Wire` (config register → all
outputs, then output register → relay states) — no extra libraries needed.

Per Waveshare's own demo code, driving an EXIO pin **HIGH turns that relay
ON**. That's what `RELAY_ACTIVE_HIGH = true` in `main.cpp` assumes. If you
flash it and the relays behave backwards (on when they should be off), flip
that constant to `false`.

Source for all of the above: the official Waveshare wiki page for this exact
board (`waveshare.com/wiki/ESP32-S3-ETH-8DI-8RO`) and its FAQ (confirms the
`0x20` I2C address).

## Tuning

Everything is at the top of `src/main.cpp`:
- `STEP_INTERVAL_MS` — gap between each relay switching (currently 2000 ms)
- `PHASE_PAUSE_MS` — pause between "all on" and "start turning off" (and
  before the cycle restarts)
- `NUM_RELAYS` — 8, matches the board

## Not included (didn't seem needed for this)

The board also has 8 digital inputs (GPIO4–11, direct — no expander), an
Ethernet port (W5500 over SPI), RS485, a buzzer, and a WS2812 RGB LED. None
of that is wired up here since the ask was just the relay chase — happy to
add any of it if useful.
