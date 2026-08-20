# AGENTS.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Firmware for an ECDL (external-cavity diode laser) v3 control system, built as two
independent Arduino sketches. There is no shared build system, package manifest, or
test suite — each `.ino` is compiled/uploaded standalone via the Arduino IDE.

- **`ECDL-v3-Software v1/v3-software-v1.ino`** — main controller: dual ADS1115 ADC
  monitoring, 20x4 I2C LCD display, and serial bridge to an external TEC (thermoelectric
  cooler) controller.
- **`PTC-voltage-interlock/PTC-voltage-interlock.ino`** — standalone hardware safety
  interlock for a separate Wavelength PTC10K-CH temperature controller. Unrelated
  hardware/pinout from the main sketch; do not assume shared state or wiring.

## Build / upload

No `arduino-cli`/PlatformIO config exists in this repo — sketches are built and
flashed via the Arduino IDE. Required libraries (install via Library Manager) for
the main sketch only:

- `Wire` (bundled)
- `Adafruit_ADS1X15`
- `LiquidCrystal_I2C`
- `SoftwareSerial` (bundled)

The interlock sketch (`PTC-voltage-interlock.ino`) uses only bundled AVR core APIs —
no external libraries.

There are no automated tests or linters; verification is hardware-in-the-loop via the
Serial Monitor (38400 baud on the main sketch) and observing the LCD/LEDs.

## `v3-software-v1.ino` architecture

**I/O map:**
- ADS1115 #0 at I2C `0x48` — channels `VTEC, ITEC, TSET, TACT`
- ADS1115 #1 at I2C `0x49` — channels `IMON, A1C1, A1C2, A1C3`
- LCD at I2C `0x27`, 20x4
- `D6`/`D7` — 3-position page-select switch, read by `getPage()` (page 0 = system
  status, 1 = ADS0 channels, 2 = ADS1 channels, 3 = TEC controller)
- `D5` — pause toggle for pages 1/2 (`checkPauseState()`); LOW = paused
- `D9` (`oePin`) — TXS0108E level-shifter output-enable, gated by `checkSupplyVoltage()`
  reading `A1` and requiring 2.97–3.63 V before enabling 3.3V-side serial
- `D10`/`D11` — `SoftwareSerial` RX/TX to the TEC controller through the level shifter
- `D13` — activity LED, held HIGH for the duration of each page refresh

**Data flow:** `loop()` polls the page/pause switches, and on a page change, pause-state
change, or the 500 ms refresh interval (`refreshInterval`), calls `printKeyChannels()`
(reads both ADS1115s, converts raw counts to engineering units, prints one CSV-style
line to `Serial` for a plotter) followed by `refreshPage()` (renders the current page
to the LCD from the same global values). `readTECController()` runs on a separate
3 s interval (`tecReadInterval`) and only when the level shifter is enabled and no
page/pause transition just happened, to avoid stealing time from the display refresh.

**Channel conversions** (in `printKeyChannels()`):
- `VTEC`/`ITEC`: bipolar op-amp scaling, `2.0 * (V - 2.5) * 1000` → mV/mA
- `TSET`/`TACT`: thermistor voltage-ratio → Kelvin via `beta = 3375` (laser-diode
  thermistor — note this differs from the TEC controller's own onboard 10k NTC, which
  uses `beta = 3950` per its datasheet; don't conflate the two)
- `IMON`: `(V / 20) * 1000` → mA
- `A1C1..A1C3`: straight V → mV

Moving averages for pages 1/2 use a fixed-size circular buffer per channel
(`ads0_history`/`ads1_history`, depth `averageLastN = 10`); pausing freezes the
displayed value at `lastAverages_*` without touching the buffers.

**TEC protocol:** `readTECController()` sends `o\r\n` and parses a single-line ASCII
reply for `Tz=`, `Tr=`, `P=`, `I=`, `D=`, `OC=`, `PW=` substrings via `strstr`/`atof`
(not a fixed-format scanf) so field order/spacing in the reply doesn't matter. A
consecutive-failure counter (`tecFailCount`) drives page 3's "no response" messaging.

**Display formatting helpers** (`formatNumber`, `formatPID`, `getChannelName`) exist to
fit values into the LCD's fixed-width 20-column layout — channel names are stored in
`PROGMEM` and copied out via `getChannelName()` to save RAM. When editing page layout
strings, preserve the fixed field widths; the LCD does not wrap or clear stale
characters on its own (note the explicit trailing-space padding throughout `refreshPage()`).

## `PTC-voltage-interlock.ino` architecture

Implements a fail-safe interlock that gates a PTC10K-CH's `ENABLE` line on the
`ACT T MON` monitor voltage staying within 0.750–1.400 V, with a *latching* fault: once
tripped, the temperature returning to range does **not** auto-clear the fault — only a
manual switch OFF→ON cycle on `D2` does. See
[`.ai/memory/results/ptc10k-ch-interlock-design.md`](.ai/memory/results/ptc10k-ch-interlock-design.md)
for the full hardware design (divider resistor values, ULN2803 LED driver rationale,
pin reservations, state table). That design doc's "Current firmware" section mirrors
this `.ino` file exactly — when changing the interlock logic, update both together, and
treat the design doc as the source of truth for *why* the state machine behaves as it
does (no hysteresis, latch-on-fault, manual-reset-required are deliberate safety choices,
not oversights).

Calibration constants to revisit before commissioning new hardware:
`ADC_REF_V` (measured 5V rail) and `R_TOP_K`/`R_BOTTOM_K` (measured divider resistors) —
see the design doc's "Practical calibration notes".
