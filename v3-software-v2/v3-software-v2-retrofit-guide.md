---
date: 2026-08-24
status: implemented, not yet bench-verified
applies-to: v3-software-v2/v3-software-v2.ino
---

# ECDL v3 -- v2 Design and Retrofit Connection Guide

Retrofit of the PTC10K-CH temperature interlock onto the existing v1 controller
hardware, replacing the deprecated TEC-8A-24V-PID-HC serial bridge.

This document is meant to stand on its own at the bench. It is the wiring and
commissioning companion to
[`.ai/memory/results/ptc10k-ch-interlock-design.md`](../.ai/memory/results/ptc10k-ch-interlock-design.md),
which remains the source of truth for *why* the interlock state machine behaves as it
does. Nothing about that state machine changed here.

---

## 1. What changed and why

The **TEC-8A-24V-PID-HC** module is deprecated and is being disconnected. The
**PTC10K-CH** becomes the temperature controller for this hardware, so its interlock has
to live on the same MCU as the display rather than on its own board.

Removing the TEC-8A gives back four pins (D9, D10, D11, A1) and, more importantly, the
up-to-1-second blocking serial wait in `readTECController()`, which was the main obstacle
to hosting a time-critical interlock in the same `loop()`.

Deleted from v1: `SoftwareSerial`, the TXS0108E output-enable logic, `checkSupplyVoltage()`,
`readTECController()`, all `tec*` state, the `rawTecResponse` String, `formatPID()`, nine
TEC PROGMEM status strings, and the `Tz`/`Tr`/`PW` serial fields.

Added: the interlock (logic carried over verbatim from `PTC-voltage-interlock.ino`),
dual-path monitor sensing, and a PTC10K-CH page in place of the old TEC page.

Build size, Arduino Nano (`arduino:avr:nano`):

| | Flash | RAM |
|---|---|---|
| v1 | 22168 B (72%) | 1559 B (76%) -- IDE warns "low memory" |
| v2 | 18876 B (61%) | 1424 B (69%) -- no warning |

**Left untouched and still deployed elsewhere:** `ECDL-v3-Software v1/v3-software-v1.ino`
and `PTC-voltage-interlock/PTC-voltage-interlock.ino`. Do not "sync" changes into them.

---

## 2. Pin map

`D2`, `D3` and `D4` are committed to other hardware on this board, so the interlock does
**not** use the standalone sketch's original A0/D2/D3/D4 pinout. Inputs are grouped in the
A0-A2 corner and every interlock output in a contiguous D9-D12 run, so the retrofit is two
headers and no existing wire moves.

| Pin | v1 use | v2 use |
|---|---|---|
| A0 | — | **ACT T MON** via divider — interlock trip path |
| A1 | 3.3 V level-shifter sense | **SET T MON** via identical divider |
| A2 | — | **MANUAL enable switch** in |
| A3 | — | spare |
| D5 | pause toggle | pages 1/2 only; pages 0/3 keep updating |
| D6 / D7 | page select | unchanged |
| D8 | — | spare (add a fault output here if wanted) |
| D9 | TXS0108E `OE` | **PTC ENABLE** out |
| D10 | SoftwareSerial RX | **OK (green) LED** |
| D11 | SoftwareSerial TX | **FAULT LED** |
| D12 | — | **FAULT LED** (parallel, same signal) |
| D13 | activity LED | unchanged |
| A4 / A5 | I2C | unchanged — **do not repurpose** |
| D0 / D1 | USB serial | unchanged, 38400 baud |

I2C bus unchanged: LCD `0x27`, ADS1115 #0 `0x48`, ADS1115 #1 `0x49`.

All fault LEDs are commanded identically and flash in sync. Any of them may be left
physically unconnected with no firmware change; to add more, extend
`PTC_FAULT_LED_PINS[]`.

---

## 3. Wiring

### 3.1 Monitor dividers (one per channel), with dual taps

ACT T MON and SET T MON can reach ~6 V, so neither may go directly to a 5 V ADC input.
Each gets its own divider, and each divider node is read **two ways**:

```text
PTC J3-3 ACT T MON
        |
      21.73k
        |
        +----------- Arduino A0        (10-bit, interlock trip decision)
        |
        +----------- ADS1115 #1 ch3    (16-bit, display / logging only)
        |
      47.31k
        |
       GND


PTC SET T MON
        |
      21.73k
        |
        +----------- Arduino A1        (10-bit)
        |
        +----------- ADS1115 #1 ch2    (16-bit)
        |
      47.31k
        |
       GND
```

Two rules behind this arrangement:

- The ADS1115 must tap the **divider node, never the raw monitor**: 0-6 V exceeds the
  ±4.096 V GAIN_ONE range (and the ADS input limit of VDD+0.3 V).
- The trip decision reads **A0 only** and never uses an I2C result. Firmware also enables
  the AVR `Wire` transaction timeout and bounds the ADS1115 conversion-ready polling
  loop. If the bus or an ADS wedges, LCD/ADS work goes offline and bounded recovery runs
  in the background while the A0 interlock continues. The ADS reading exists so a wiring
  or calibration problem shows up as a visible disagreement on page 3, not so the
  interlock can use it.

ADS1115 input impedance is ~6 MΩ, so tapping the same node loads the divider negligibly.

Resolution comparison at ACT T MON, near 25 °C:

| Path | Per LSB at the monitor | Approx. per LSB in °C |
|---|---|---|
| A0, 10-bit | ~7.24 mV | ~0.16 °C |
| ADS1115, GAIN_ONE | ~0.19 mV | ~0.004 °C |

### 3.2 Manual enable switch

```text
+5 V
 |
manual SPST switch
 |
 +---------- Arduino A2
 |
10k
 |
GND
```

Switch ON -> A2 HIGH. Switch OFF -> A2 LOW. The 10k gives a defined LOW with the switch
open. **OFF is also the deliberate fault-reset state** — it is the only way to clear a
latched fault.

### 3.3 PTC ENABLE

```text
Arduino D9 ---- 1k ---- PTC J3-1 ENABLE
                          |
                        10k
                          |
                         GND
```

The 1k limits fault current between devices. The **10k pulldown is not optional**: it is
what holds ENABLE LOW while the Arduino is unpowered, resetting, or driving a
high-impedance pin.

### 3.4 Indicator LEDs

D10 (green OK) and D11/D12 (fault) may drive small indicator LEDs directly through
suitable series resistors. For bright 15-20 mA panel LEDs, or several per output, drive
them through a **ULN2803**-style array — see the design doc's LED current-limit section:

```text
Arduino fault output ---> ULN2803 input

+5 V ---- resistor ---- LED ---- ULN2803 output ---- GND
```

### 3.5 Grounding

All grounds must share one reference, using the PTC's low-current **COMMON** for signal
reference:

```text
PTC10K-CH COMMON + 5 V regulator GND + Arduino GND + 12 V supply GND
        |
        +---- common ground
```

---

## 4. Retrofit procedure

1. **Power down** everything and disconnect the PTC10K-CH ENABLE line before starting.
2. **Remove the TEC-8A harness**: the serial pair to D10/D11, the TXS0108E `OE` wire on
   D9, and the 3.3 V sense wire on A1. Remove the TXS0108E level-shifter connections
   entirely — a leftover stub on D9/D10/D11 will now fight the interlock outputs.
3. **Build the two dividers** (21.73k top / 47.31k bottom each) close to the Arduino, and
   land their nodes on A0 (ACT) and A1 (SET).
4. **Run the second tap** from each divider node to ADS1115 #1 ch3 (ACT) and ch2 (SET) —
   this is how the bench unit was actually wired; ch1 stays spare (`A1C1`).
5. **Wire the manual switch** to A2 with its 10k pulldown to GND.
6. **Wire ENABLE**: D9 through 1k to J3-1, with the 10k pulldown at the PTC end. Leave the
   PTC end disconnected until step 7 of the verification below.
7. **Wire the indicators**: green OK on D10, fault on D11 (and D12 if a second group is
   fitted), through ULN2803 drivers if the LEDs are bright.
8. **Confirm the common ground** per §3.5.
9. **Flash** `v3-software-v2/v3-software-v2.ino` (Arduino IDE, or
   `arduino-cli compile --fqbn arduino:avr:nano v3-software-v2`). Required libraries:
   `Wire`, `Adafruit_ADS1X15` **>= 2.0** (the I2C-recovery code uses the async
   `startADCReading()`/`conversionComplete()`/`getLastConversionResults()` API, which does
   not exist in the older 1.x releases), `LiquidCrystal_I2C`. `SoftwareSerial` is no longer
   needed.

### Pre-power checklist

- [ ] 10k pulldown present at the PTC ENABLE end, measured.
- [ ] Divider resistors measured, and `R_TOP_K` / `R_BOTTOM_K` in the sketch updated to
      the measured values.
- [ ] `ADC_REF_V` set to the measured 5 V rail.
- [ ] No remaining TXS0108E / TEC-8A wiring on D9, D10, D11 or A1.
- [ ] A4/A5 untouched; LCD and both ADS1115s still respond.
- [ ] Manual switch reads HIGH when ON and LOW when OFF, at the pin.
- [ ] PTC ENABLE still physically disconnected (reconnect in verification step 7).
- [ ] `Adafruit_ADS1X15` library is >= 2.0 and the `arduino:avr` board package is recent
      enough to provide `Wire.setWireTimeout()`/`getWireTimeoutFlag()`/
      `clearWireTimeoutFlag()` (added ~2021); an older cached copy of either will fail to
      compile, not fail silently. `arduino-cli compile --fqbn arduino:avr:nano
      v3-software-v2` is a quick way to confirm before touching hardware.

---

## 5. Behavior reference

### 5.1 Interlock state table

Unchanged from the design doc. Window is **0.750-1.400 V**, no hysteresis, fault latches
after **1000 ms** of continuous out-of-range reading, and only a manual OFF->ON cycle
clears it.

| Manual switch | ACT T MON | Fault latch | PTC ENABLE | Green LED | Fault LEDs |
|---|---|---|---|---|---|
| OFF | in range | cleared | LOW | OFF | OFF |
| OFF | out of range | cleared | LOW | OFF | solid ON |
| ON | in range, no previous fault | clear | HIGH | ON | OFF |
| ON, previously qualified | out of range < 1 s | not yet set | HIGH | ON | OFF |
| ON | out of range >= 1 s continuous | set | LOW | OFF | flash 10 Hz |
| ON | recovered after fault | remains set | LOW | OFF | flash 1 Hz |
| OFF -> ON, in range | | clear | HIGH | ON | OFF |
| OFF -> ON, out of range < 1 s | | clear | LOW | OFF | solid ON |

Startup and every OFF->ON edge discard the sample buffer and require a fresh, complete,
in-range 9-sample median window before ENABLE may go HIGH.

### 5.2 Interlock timing on shared hardware

This is the one behavioral difference from the standalone sketch. There, one ADC sample
was taken per `loop()` pass. Here `loop()` blocks: eight ADS1115 single-shot reads take
~70 ms and an LCD refresh tens of ms. So sampling is nominally gated to a **5 ms**
interval by `serviceInterlock()`, which is also called from inside the ADS read loops,
between LCD characters, and during background recovery. I2C transactions have a 25 ms
timeout and ADS conversion polling has a 50 ms timeout; after either failure, normal I2C
work stops so the analog interlock can continue without waiting on the bus.

Consequences:

- Outside serial emission, the 9-sample median window is typically **~45 ms** and the
  10 Hz fault flash remains responsive during ADS/LCD work.
- A sustained excursion still latches within ~1 s regardless of what the display is doing.
- If an ADS1115 fails to initialise, setup returns into the normal loop with I2C marked
  offline. The interlock starts normally and background recovery begins; PTC protection
  depends on neither I2C nor the ADS1115.

**Automatic I2C recovery.** While I2C is offline, firmware retries every 2 s. Each attempt
releases the AVR TWI peripheral, supplies up to nine SCL recovery pulses and a STOP,
restarts `Wire`, and probes the LCD plus both ADS1115 addresses. A failed transaction is
still bounded by 25 ms, and `serviceInterlock()` runs between probes. Once all three
devices acknowledge, the ADS library objects are rebound and monitoring resumes without
resetting the Arduino, clearing the fault latch, or changing PTC ENABLE. Serial reports
`I2C recovered; LCD/ADS monitoring resumed` and `PI2C` returns to 1.

The LCD library's full initialisation sequence blocks for about 1 s, so firmware never
runs it while PTC ENABLE is HIGH. If the LCD retained power, ordinary writes generally
resume after bus recovery. If the LCD itself power-cycled and needs full reinitialisation,
that step is deferred until ENABLE is already LOW; ADS telemetry and the A0 interlock do
not wait for it. A peripheral that remains electrically failed or holds a bus line LOW
will remain offline and be retried—it may still require power-cycling that peripheral,
but not the Arduino or PTC interlock.

**Serial timing quirk.** The CSV record is longer than the AVR hardware-serial transmit
buffer. At 38400 baud, `Serial.print()` therefore blocks for tens of milliseconds once
per 500 ms refresh while the UART drains. `serviceInterlock()` cannot run during that
short burst, so an individual sample and a fault-LED transition may be late by roughly
one telemetry burst. The elapsed-time fault debounce still applies and resumes on the
next service call. This is accepted for the present firmware; reducing telemetry,
raising the baud rate, or making transmission non-blocking would remove the jitter.

### 5.3 Page 3 -- PTC10K-CH page (replaces the TEC page)

```text
ACT 1.021V 1.0208V
SET 1.005V 1.0049V
T 24.9C/25.2C ~UNCAL
EN=ON  OK
```

- Left column: the A0/A1 10-bit reading — what the interlock actually trips on.
- Right column: the ADS1115 16-bit reading of the same node. A persistent disagreement
  means a wiring or calibration problem, not noise.
- Row 2: derived °C, flagged `~UNCAL` (see §6).
- Row 3: `EN=ON/OFF` plus state — `OK`, `WARMUP`, `MANUAL OFF`, `UNQUALIFIED`,
  `FAULT 10Hz`, `FAULT LATCH`.

Other display changes: page 2's `A1C2`/`A1C3` are renamed **`SETM`/`ACTM`** and now read in
reconstructed monitor mV (the divider is undone in firmware); `A1C1` stays spare and
unrenamed. Page 0's D6/D7 pin-state
debug line is replaced by a one-line interlock summary.

### 5.4 Serial CSV (38400 baud)

`Tz`/`Tr`/`PW` are gone. New fields, alongside the unchanged VTEC/ITEC/TSET/TACT/IMON and
the raw/volts block:

| Field | Meaning |
|---|---|
| `PACTV` | ACT T MON volts, A0 path (the trip value) |
| `PACTADS` | ACT T MON volts, ADS1115 path |
| `PSETV` | SET T MON volts, A1 path |
| `PSETADS` | SET T MON volts, ADS1115 path |
| `PACTR` | ACT thermistor ohms (bias-current only — trustworthy) |
| `PACTC`, `PSETC` | derived °C (**uncalibrated**, see §6) |
| `PEN` | 1 = ENABLE asserted |
| `PFLT` | 1 = fault latched |
| `PI2C` | 1 = LCD/ADS bus healthy; 0 = offline with recovery retries active |
| `ACTMR`/`ACTMV`, `SETMR`/`SETMV` | raw counts and volts at the ADS taps |

---

## 6. Calibration

Two independent things need calibrating. Do them in this order.

### 6.1 Voltage path (do before commissioning)

1. Measure the regulated 5 V rail; put the measured value in `ADC_REF_V`.
2. Measure the two divider resistors; update `R_TOP_K` and `R_BOTTOM_K`.
3. Apply known voltages at the divider input and compare `PACTV` against a multimeter.
4. Confirm the boundaries land where expected: with nominal values, 0.750 V is ADC ~104
   and 1.400 V is ADC ~193 at A0.

The 0.750/1.400 V limits themselves do not change.

### 6.2 Temperature readout -- currently UNCALIBRATED

The sensor bias current is **confirmed at 100 µA**. `PTC_BETA` is **assumed at 3950** and
has *not* been measured on this unit, which is why the °C readout carries the `~UNCAL`
marker and why voltage — not temperature — is what the interlock trips on.

With the assumed beta, the trip window works out to roughly:

| ACT T MON | Thermistor | Approx. °C (assumed beta) |
|---:|---:|---:|
| 0.750 V | 7.50 kΩ | ~31.6 °C |
| 1.000 V | 10.00 kΩ | 25.0 °C |
| 1.400 V | 14.00 kΩ | ~17.6 °C |

Treat those °C figures as indicative only until beta is fitted.

To calibrate: let the loop settle at two or three well-separated stable temperatures, and
at each point record `PACTR` (the resistance, which needs only the confirmed bias current
and is therefore already trustworthy) against the PTC's own front-panel temperature. Fit

```text
beta = ln(R1/R2) / (1/T1 - 1/T2)      T in kelvin
```

Put the result in `PTC_BETA`, re-verify against a third point, then delete the `~UNCAL`
marker from page 3 and drop the "ASSUMED" comment in the sensor-model block. If the fit is
poor across a wide span, switch to Steinhart-Hart coefficients for the actual sensor part.

---

## 7. Known behavior to accept

**USB connection blips PTC ENABLE.** Opening the serial port from a host resets an
Uno/Nano. That drops ENABLE. With the manual switch still ON, the interlock then discards
its buffer, re-qualifies from a fresh median window, and re-enables in roughly 50 ms with
no operator action — the standalone sketch's documented startup behavior. The difference
is that *this* board normally has a USB host attached, so every Serial Monitor connection
briefly interrupts temperature control.

Options, in order of preference: accept it and avoid connecting the monitor during a
measurement run; or defeat auto-reset on the board (cut the RESET-EN trace / fit a
capacitor on RESET) if the blip is unacceptable, accepting that flashing then requires a
manual reset.

**Auto-enable at power-up.** If power is applied with the manual switch already ON and the
temperature in range, ENABLE goes HIGH after the ~50 ms qualification window without
operator action. This is by design, and unchanged from the standalone interlock.

---

## 8. Verification (hardware-in-the-loop)

There are no automated tests in this repo. Verify in this order, with the PTC ENABLE line
still disconnected for steps 1-6.

1. **Compile.** No `SoftwareSerial` reference survives; flash/RAM match §1.
2. **Voltage path.** Feed the A0 divider node from a bench PSU or pot. On the serial line,
   check `PACTV` against a multimeter, check `PACTV` vs `PACTADS` agreement, and confirm
   the 0.750 V / 1.400 V boundaries.
3. **Interlock behavior.** Walk the §5.1 state table: latch after ~1 s continuous
   out-of-range; 10 Hz flash while out of range; 1 Hz after recovery with ENABLE still
   LOW; manual OFF->ON as the only reset; solid-ON fault LEDs with the switch OFF and the
   input bad.
4. **Fail-safe states.** ENABLE stays LOW through power-up, reset, unpowered, the first
   fresh median window after OFF->ON, and OFF->ON while already out of range.
5. **Jitter check.** Hold a fault while cycling pages and toggling pause. Confirm pages
   0/3 and serial continue updating while D5 is LOW, the flash remains responsive during
   LCD/ADS work, and the trip still happens at about 1 s. A brief flash jitter coincident
   with each serial CSV burst is the documented quirk above.
6. **ADS/I2C failure and recovery.** Boot with ADS1115 #1 absent and confirm the
   interlock still qualifies, trips, and flashes normally while recovery retries. Restore
   the device without resetting the Arduino and confirm the recovery message, `PI2C=1`,
   and resumed ADS telemetry. Repeat by interrupting the bus at runtime while ENABLE is
   HIGH: `PI2C` should become 0, A0 must keep protecting the PTC, and recovery must not
   blip ENABLE or clear a latched fault. If the LCD itself was power-cycled, confirm its
   full reinitialisation occurs only after ENABLE is LOW.
7. **On the real controller.** Connect PTC ENABLE (1k series, 10k pulldown) and re-run
   steps 3-4 against the PTC10K-CH itself. Then collect the resistance/temperature pairs
   for §6.2 and fit beta.
