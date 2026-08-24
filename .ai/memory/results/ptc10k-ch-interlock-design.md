---
date: 2026-08-24
status: settled
---

Hardware/firmware design for an Arduino interlock that gates a Wavelength PTC10K-CH's ENABLE line on ACT T MON voltage (0.750-1.400 V), with a latching fault requiring manual switch OFF->ON reset before re-enable. Startup and every manual OFF->ON transition require a fresh, complete, in-range median window before ENABLE can go HIGH. Once enabled, the ADC read is noise-rejected with a rolling 9-sample median, and a fault only latches after 1 s of continuous out-of-range readings (debounce on time, not a hysteresis band on the voltage thresholds). Implemented and matches `PTC-voltage-interlock/PTC-voltage-interlock.ino`.

# PTC10K-CH Hardware Temperature-Interlock Design Note

## Purpose

This note captures the agreed design for a simple hardware/Arduino interlock around a Wavelength **PTC10K-CH** temperature controller.

The design monitors **ACT T MON** and only allows the controller's **ENABLE** input to go high when the measured monitor voltage is within the accepted range.

This file is intended to be self-contained so the design can be continued in another ChatGPT thread, Codex session, or engineering notebook.

---

## Functional requirements

### Normal operating window

The accepted temperature-monitor voltage is:

- **LOW limit:** 0.750 V
- **HIGH limit:** 1.400 V

There is deliberately **no hysteresis**.

### Enable logic

The PTC may enter its enabled state only when:

1. the manual enable switch is ON,
2. ACT T MON is within 0.750–1.400 V,
3. no fault is currently latched.

After startup or manual OFF, the firmware discards earlier samples and requires a
fresh, complete median window with an in-range reading before it sets
`ENABLE_QUALIFIED` and permits ENABLE to go HIGH.
Once qualified, ENABLE remains HIGH during a sub-1-second out-of-range debounce
window; a sustained excursion latches the fault and drives it LOW. In stateful
logical form:

```text
PTC_ENABLE = MANUAL_ENABLE && ENABLE_QUALIFIED && !FAULT_LATCHED
```

`ENABLE_QUALIFIED` is cleared by manual OFF and is set only by a complete,
median-filtered in-range reading. It remains set during the debounce window so a
brief excursion does not interrupt operation.

### ADC noise rejection and fault debounce

A single noisy ADC sample (mux/conversion noise, EMI) must not be able to trip the
fault on its own, since the fault is a latch that requires a manual reset. Two
independent filters guard against this, neither of which touches the 0.750/1.400 V
trip thresholds themselves:

1. **Rolling median.** Each `loop()` pass takes one fresh `analogRead()` and pushes
   it into a 9-sample circular buffer; the ADC value used for the voltage
   reconstruction is the median of that buffer, not the raw instantaneous sample.
   A single outlier sample is rejected outright as long as it isn't more than 4 of
   the last 9 readings. Until the buffer has collected a full 9 samples at power-up
   or after an OFF->ON switch transition, the reading is treated as fail-safe
   not-OK, so ENABLE stays LOW through that brief qualification period.
2. **Time debounce on the latch.** After the median buffer is full, the
   (median-filtered) reading must be
   continuously out of range for **1000 ms** before the fault actually latches.
   If ENABLE was already qualified by an in-range reading, it remains HIGH during
   that debounce window. At startup or after manual OFF, an out-of-range reading
   does not qualify ENABLE, so it remains LOW while the same timer runs. If the
   reading returns in-range before 1 s elapses, the timer resets and nothing latches.

This means a genuine, sustained ACT T MON excursion still disables the PTC within
about a second, at the cost of that ~1 s of added response latency versus the
previous design's near-instant trip. Given the PTC/thermistor thermal time
constants involved, this delay is expected to be negligible for the interlock's
purpose — call this assumption out explicitly if the hardware ever changes to
something with a much faster thermal fault mode.

### Fault behavior

If the manual switch is ON and ACT T MON leaves the allowed range continuously for
at least 1 second (after the rolling-median filter):

- PTC ENABLE is driven LOW,
- green/OK LED turns OFF,
- the fault is **latched**,
- all connected red fault LEDs flash at **10 Hz** while ACT T MON remains out of range.

If ACT T MON subsequently returns to 0.750–1.400 V:

- the PTC remains disabled,
- the fault remains latched,
- red fault LEDs flash at **1 Hz**.

Returning to the valid voltage range does **not** automatically re-enable the PTC.

### Manual reset behavior

To clear a latched fault:

1. move the manual enable switch OFF, pulling the manual input to GND,
2. then move the switch ON again.

When the switch is OFF:

- PTC ENABLE is LOW,
- the fault latch is cleared, and the debounce timer is also cleared (so a
  timer that was already running before the switch went OFF can't fire a
  stale, instant latch right after the switch goes back ON),
- if ACT T MON is still out of range, fault LEDs remain **solid ON**,
- if ACT T MON is in range, fault LEDs are OFF.

When the switch is switched ON again:

- if ACT T MON is in range, the PTC is enabled,
- if ACT T MON is still out of range, ENABLE remains LOW, the debounce timer starts
  fresh, and a fault latches after another 1 s of continuous out-of-range reading.

This gives an intentional operator-reset requirement after a fault.

---

## Power arrangement

Existing system:

- main PTC10K-CH supply: **12 V or higher**
- separate regulated **5 V rail** derived from the same supply
- Arduino powered from this regulated 5 V rail

All logic grounds must share a common reference:

```text
PTC10K-CH COMMON
5 V regulator GND
Arduino GND
12 V supply GND
        |
        +---- common ground
```

Use the PTC's low-current **COMMON** connection for signal reference.

---

## PTC10K-CH signals used

Relevant J3 signals:

- **J3-1 ENABLE** -> controlled by Arduino
- **J3-2 COMMON** -> signal ground
- **J3-3 ACT T MON** -> monitored by Arduino
- J3-5 COMMON may also be used as the signal common where appropriate

Design assumptions from the PTC10K-CH documentation used during this design:

- ENABLE is disabled below approximately **1.45 V**
- ENABLE is enabled above approximately **3.4 V**
- ACT T MON can span approximately **0–6 V**
- ACT T MON has approximately **1 kOhm output impedance**

A 5 V Arduino digital output therefore comfortably drives the PTC ENABLE logic level.

Reference document:

`https://www.teamwavelength.com/download/Datasheets/ptcxk-ch.pdf`

---

## ACT T MON to Arduino A0

Because ACT T MON can reach approximately 6 V, it should not be connected directly to a 5 V Arduino ADC input.

No clamp diode is assumed to be available.

A resistor divider is used instead.

### Measured resistor values

Actual measured values:

- upper resistor, ACT T MON -> A0: **21.73 kOhm**
- lower resistor, A0 -> GND: **47.31 kOhm**

The PTC monitor's approximately 1 kOhm output impedance is included in the conversion.

```text
PTC J3-3 ACT T MON
        |
      21.73k
        |
        +---------- Arduino A0
        |
      47.31k
        |
       GND
```

Effective upper resistance:

```text
21.73 kOhm + ~1.00 kOhm PTC output impedance
= ~22.73 kOhm
```

### Divider ratio

Using the above values:

```text
V_A0 / V_ACT = 0.67547116
```

or approximately:

```text
V_A0 = 0.67547 * V_ACT
```

Therefore:

| ACT T MON | Approx. A0 voltage |
|---:|---:|
| 0.750 V | 0.5066 V |
| 1.000 V | 0.6755 V |
| 1.400 V | 0.9457 V |
| 6.000 V | 4.0528 V |

The 6 V monitor maximum therefore produces only about **4.05 V** at A0, safely below 5 V under the assumed normal PTC output range.

This resistor divider protects against the specified monitor range. It is not intended as protection against arbitrary high-voltage wiring faults or large transients.

---

## ADC scaling

Assuming a classic 5 V ATmega328P Arduino using a 10-bit ADC:

```text
ADC = 0 ... 1023
```

The code reconstructs the PTC-side ACT T MON voltage:

```cpp
float vA0 = adc * ADC_REF_V / 1023.0f;
float vAct = vA0 / DIVIDER_RATIO;
```

With a nominal 5.000 V ADC reference:

- 0.750 V ACT T MON corresponds to ADC ~= **103.6**
- 1.400 V ACT T MON corresponds to ADC ~= **193.5**
- one ADC count corresponds to approximately **7.24 mV** at ACT T MON

The code should keep the limits expressed in reconstructed ACT T MON volts rather than hard-coded ADC counts.

This makes calibration simple: measure the actual Arduino 5 V rail and update only:

```cpp
const float ADC_REF_V = 5.000f;
```

For example, if the rail is actually 4.947 V, use:

```cpp
const float ADC_REF_V = 4.947f;
```

The 0.750 V and 1.400 V limits do not need to change.

---

## Manual enable switch

Recommended wiring:

```text
+5 V
 |
manual SPST switch
 |
 +---------- Arduino D2
 |
10 kOhm
 |
GND
```

Therefore:

- switch ON -> D2 HIGH
- switch OFF -> D2 LOW

The 10 kOhm resistor ensures the input has a defined LOW state when the switch is open.

The OFF state is also the deliberate fault-reset state.

---

## Arduino to PTC ENABLE

Recommended connection:

```text
Arduino D3 ---- 1 kOhm ---- PTC J3-1 ENABLE
                              |
                            10 kOhm
                              |
                             GND
```

Purpose of the resistors:

- **1 kOhm series resistor:** limits fault/current between devices
- **10 kOhm pull-down:** forces the PTC ENABLE signal LOW while the Arduino is unpowered, resetting, or its pin is high-impedance

Normal behavior:

- D3 HIGH -> approximately 5 V -> PTC enabled
- D3 LOW -> approximately 0 V -> PTC disabled

---

## Green OK LED

The green LED indicates actual enabled operation.

Behavior:

- green ON only when PTC ENABLE is HIGH
- green OFF in every disabled or fault state

If driven directly from the Arduino, use an appropriate series resistor and remain within the board's GPIO current limits.

---

## Multiple fault LEDs

The firmware is designed so any number of extra fault-LED outputs may be physically connected or left unused **without changing the code**.

Suggested available fault-output pins on a classic Uno/Nano:

```text
D5 D6 D7 D8 D9 D10 D11 D12 D13
A1 A2 A3 A4 A5
```

Reserved:

- A0 -> ACT T MON ADC
- D2 -> manual switch
- D3 -> PTC ENABLE
- D4 -> green OK LED
- D0/D1 -> leave free for UART / USB serial / programming
- A6/A7, where present on some Nano boards, are analog-input-only and should not be treated as general digital outputs

All fault LEDs are commanded identically, so they flash in sync.

Leaving an output physically unconnected requires no firmware changes.

---

## IMPORTANT: LED current limits

Do **not** directly run many 15–20 mA LEDs from the ATmega328P GPIO pins simultaneously.

Although an individual GPIO can be used at modest LED current, the MCU has:

- per-pin current limits,
- grouped-port current limits,
- a total chip current limit.

For example:

```text
14 LEDs * 15 mA = 210 mA
14 LEDs * 20 mA = 280 mA
```

This is not suitable as direct GPIO loading.

### Recommended solution for bright fault LEDs

Use external low-side transistor drivers.

A convenient choice is:

- **2 x ULN2803 / ULN2803A / equivalent 8-channel transistor-array drivers**

Architecture:

```text
Arduino fault output
        |
        v
ULN2803 input

+5 V ---- LED resistor ---- LED ---- ULN2803 output
                                      |
                                     GND
```

Advantages:

- LEDs draw their current from the external regulated 5 V supply
- Arduino GPIO only drives the transistor-array inputs
- any LED may be connected or omitted with no firmware changes
- many bright LEDs can operate simultaneously

For 14 fault LEDs at 20 mA each:

```text
14 * 20 mA = 280 mA
```

Allow additional current for the Arduino, green LED, driver losses, etc.

A **5 V regulator rated for at least ~500 mA** is a sensible minimum if it supplies the Arduino plus all 14 bright fault LEDs, assuming no other significant loads.

A larger margin is preferable.

---

## State table

| Manual switch | ACT T MON | Fault latch | PTC ENABLE | Green LED | Fault LEDs |
|---|---|---|---|---|---|
| OFF | in range | cleared | LOW | OFF | OFF |
| OFF | out of range | cleared | LOW | OFF | solid ON |
| ON | in range, no previous fault | clear | HIGH | ON | OFF |
| ON, previously qualified | out of range < 1 s (debounce window) | not yet set | HIGH | ON | OFF |
| ON | out of range >= 1 s continuous | set | LOW | OFF | flash 10 Hz |
| ON | recovered after fault | remains set | LOW | OFF | flash 1 Hz |
| OFF after a fault | in range | cleared | LOW | OFF | OFF |
| OFF after a fault | out of range | cleared | LOW | OFF | solid ON |
| OFF -> ON, in range | clear | HIGH | ON | OFF |
| OFF -> ON, out of range < 1 s | clear | LOW | OFF | solid ON |
| OFF -> ON, out of range >= 1 s continuous | set | LOW | OFF | flash 10 Hz |

---

## State-machine reasoning

The key design choice is that returning to the valid temperature range does not automatically restart the PTC.

Fault sequence:

```text
NORMAL
  |
ACT T MON exits 0.75-1.4 V (median-filtered)
  |
  v
DEBOUNCE WINDOW (up to 1 s)
EN stays HIGH
  |
still out of range after 1 s continuous
  |
  v
FAULT LATCHED
EN = LOW
fault LEDs = 10 Hz
  |
temperature recovers
  |
  v
FAULT STILL LATCHED
EN = LOW
fault LEDs = 1 Hz
  |
manual switch -> OFF/GND
  |
  v
FAULT LATCH CLEARED
EN remains LOW
  |
manual switch -> ON
  |
  +-- ACT in range --> EN HIGH
  |
  +-- ACT out of range --> EN remains LOW
                           fault latches after 1 s continuous
```

This prevents an uncontrolled automatic restart after a temperature excursion.

---

## Current firmware

```cpp
// ============================================================
// PTC10K-CH Temperature Interlock
// Classic 5 V Arduino Uno / Nano (ATmega328P)
// ============================================================

#include <string.h>

const uint8_t PIN_TEMP   = A0;
const uint8_t PIN_MANUAL = 2;
const uint8_t PIN_ENABLE = 3;
const uint8_t PIN_LED_OK = 4;

// All remaining sensible output pins.
// D0/D1 are deliberately reserved for USB/serial programming.
// A6/A7 on a classic Nano are analog-input-only.
const uint8_t FAULT_LED_PINS[] = {
  5, 6, 7, 8, 9, 10, 11, 12, 13,
  A1, A2, A3, A4, A5
};

const uint8_t NUM_FAULT_LEDS =
  sizeof(FAULT_LED_PINS) / sizeof(FAULT_LED_PINS[0]);


// ---------------- ADC / divider ----------------

// Measure your actual regulated 5 V rail if you want better accuracy.
const float ADC_REF_V = 5.000;

// External divider:
// ACT T MON -- 22k -- A0 -- 47k -- GND
//
// PTC ACT T MON itself has ~1k output impedance,
// so effective upper resistance is 22k + 1k.

const float R_TOP_K     = 21.73;
const float R_PTC_OUT_K = 1.00;
const float R_BOTTOM_K  = 47.31;

const float DIVIDER_RATIO =
  R_BOTTOM_K / (R_BOTTOM_K + R_TOP_K + R_PTC_OUT_K);

const float V_LOW  = 0.750;
const float V_HIGH = 1.400;


// ---------------- ADC noise rejection ----------------

// Rolling median over the last N raw ADC samples (one new sample per
// loop pass). Rejects single-sample spikes (ADC noise, EMI) without
// adding hysteresis to the V_LOW/V_HIGH trip thresholds themselves.
// Window is a time span, not a fixed duration: it scales with how
// fast loop() runs, so it stays negligible only as long as loop()
// stays unblocked (no added delay()s).
const uint8_t MEDIAN_WINDOW = 9;

int sampleBuf[MEDIAN_WINDOW];
uint8_t sampleIdx = 0;
bool bufferFull = false;

// Fault only latches after this many consecutive milliseconds of
// out-of-range readings, so a brief spike that slips past the median
// filter still can't trip the fault. A genuine, sustained excursion
// still latches within ~1 s.
const unsigned long FAULT_DEBOUNCE_MS = 1000;

bool outOfRangeActive = false;
unsigned long outOfRangeStart = 0;

// Becomes true only after a complete median window reports an
// in-range temperature while the manual switch is ON. This prevents
// startup or a manual reset from enabling an already-faulty system.
bool enableQualified = false;

// Tracks the manual switch edge so each OFF->ON cycle must collect a
// fresh median window before it can qualify ENABLE.
bool manualWasON = false;


// Insertion sort a copy of the buffer and return the middle element.
int median9(const int *buf)
{
  int sorted[MEDIAN_WINDOW];
  memcpy(sorted, buf, sizeof(sorted));

  for (uint8_t i = 1; i < MEDIAN_WINDOW; i++) {
    int key = sorted[i];
    int j = i;
    while (j > 0 && sorted[j - 1] > key) {
      sorted[j] = sorted[j - 1];
      j--;
    }
    sorted[j] = key;
  }

  return sorted[MEDIAN_WINDOW / 2];
}


// ---------------- Fault state ----------------

bool faultLatched = false;


// Set every fault LED output together.
void setFaultLEDs(bool on)
{
  for (uint8_t i = 0; i < NUM_FAULT_LEDS; i++) {
    digitalWrite(FAULT_LED_PINS[i], on ? HIGH : LOW);
  }
}


void setup()
{
  pinMode(PIN_MANUAL, INPUT);

  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_LED_OK, OUTPUT);

  for (uint8_t i = 0; i < NUM_FAULT_LEDS; i++) {
    pinMode(FAULT_LED_PINS[i], OUTPUT);
  }

  // Fail-safe startup state
  digitalWrite(PIN_ENABLE, LOW);
  digitalWrite(PIN_LED_OK, LOW);
  setFaultLEDs(false);
}


void loop()
{
  // ==========================================================
  // Read ACT T MON (rolling median of last MEDIAN_WINDOW samples)
  // ==========================================================

  sampleBuf[sampleIdx] = analogRead(PIN_TEMP);
  sampleIdx++;
  if (sampleIdx >= MEDIAN_WINDOW) {
    sampleIdx = 0;
    bufferFull = true;
  }

  // Until the buffer has a full window of real samples, treat the
  // reading as not-OK (fail-safe: ENABLE stays LOW, its power-up
  // default, through the brief startup warm-up).
  bool tempOK = false;
  float vAct = 0.0;

  if (bufferFull) {
    int adc = median9(sampleBuf);

    float vA0 =
      adc * ADC_REF_V / 1023.0;

    vAct =
      vA0 / DIVIDER_RATIO;

    tempOK =
      (vAct >= V_LOW) &&
      (vAct <= V_HIGH);
  }

  bool manualON =
    (digitalRead(PIN_MANUAL) == HIGH);


  // ==========================================================
  // MANUAL SWITCH OFF
  // ==========================================================

  if (!manualON)
  {
    // Pulling manual switch to GND resets the fault latch and the
    // debounce timer, so a stale timer can't fire an instant latch
    // on the next switch-ON.
    faultLatched = false;
    outOfRangeActive = false;
    enableQualified = false;
    manualWasON = false;

    // PTC always disabled while manual switch is OFF.
    digitalWrite(PIN_ENABLE, LOW);
    digitalWrite(PIN_LED_OK, LOW);

    // If temperature is still bad, fault LEDs stay solid ON.
    setFaultLEDs(!tempOK);

    return;
  }


  // Discard samples from before this switch-ON edge. A stale in-range
  // median must not briefly enable a system whose current input is bad.
  if (!manualWasON) {
    manualWasON = true;
    sampleIdx = 0;
    bufferFull = false;
  }


  // Do not start the debounce timer or enable the PTC until a full
  // median window is available. setup() has already driven ENABLE LOW.
  if (!bufferFull) {
    digitalWrite(PIN_ENABLE, LOW);
    digitalWrite(PIN_LED_OK, LOW);
    setFaultLEDs(false);
    return;
  }


  // ==========================================================
  // MANUAL SWITCH ON
  // ==========================================================

  // Fault only latches after FAULT_DEBOUNCE_MS of continuous
  // out-of-range readings; a single bad reading (or one the median
  // filter missed) that clears on the next pass never latches.
  if (!tempOK) {
    if (!outOfRangeActive) {
      outOfRangeActive = true;
      outOfRangeStart = millis();
    } else if (millis() - outOfRangeStart >= FAULT_DEBOUNCE_MS) {
      faultLatched = true;
    }
  } else {
    outOfRangeActive = false;
    if (!faultLatched) {
      enableQualified = true;
    }
  }


  // ==========================================================
  // NORMAL ENABLED STATE
  // ==========================================================

  // Once an in-range reading has qualified the enabled state, gate
  // only on faultLatched so a brief out-of-range reading inside the
  // debounce window does not drop ENABLE. Before qualification (at
  // startup or after manual OFF), ENABLE remains LOW.
  if (!faultLatched)
  {
    if (enableQualified) {
      digitalWrite(PIN_ENABLE, HIGH);
      digitalWrite(PIN_LED_OK, HIGH);
      setFaultLEDs(false);
    } else {
      digitalWrite(PIN_ENABLE, LOW);
      digitalWrite(PIN_LED_OK, LOW);
      setFaultLEDs(!tempOK);
    }

    return;
  }


  // ==========================================================
  // FAULT-LATCHED STATE
  // ==========================================================

  digitalWrite(PIN_ENABLE, LOW);
  digitalWrite(PIN_LED_OK, LOW);


  if (!tempOK)
  {
    // Currently outside 0.75–1.4 V
    //
    // 10 Hz:
    // full period = 100 ms
    // half period = 50 ms

    bool flashState =
      ((millis() / 50) % 2) == 0;

    setFaultLEDs(flashState);
  }
  else
  {
    // Temperature has recovered,
    // but fault remains latched.
    //
    // 1 Hz:
    // full period = 1000 ms
    // half period = 500 ms

    bool flashState =
      ((millis() / 500) % 2) == 0;

    setFaultLEDs(flashState);
  }
}
```

---

## Numerical / overflow conclusions

For a classic ATmega328P Arduino:

- `analogRead()` produces only 0–1023, safely inside a 16-bit `int`
- divider calculations involve values near 0–7.5 V, nowhere near floating-point overflow
- resistor values are trivial for 32-bit AVR floating point
- LED array index count is only 14, safely inside `uint8_t`
- `millis()` uses an unsigned 32-bit counter and wraps after approximately 49.7 days

The `millis()` rollover does not threaten the fault latch or ENABLE logic in this implementation. It may cause one irregular LED flash at the rollover instant before normal flashing continues.

The relevant numerical limitation is ADC resolution and calibration, not overflow.

---

## No hysteresis (but there is a time debounce — these are different things)

There is deliberately no hysteresis on the voltage thresholds in this design.

The valid/recovery range is exactly:

```text
0.750 V <= ACT T MON <= 1.400 V
```

After a latched fault, entering this range changes the fault indication from 10 Hz to 1 Hz, but does **not** re-enable the PTC.

Re-enable still requires a manual OFF -> ON reset cycle.

Separately, the firmware does debounce the *fault-latching decision* in time (see
"ADC noise rejection and fault debounce" above): a reading must be continuously
out of range for 1 s before it latches. This is not hysteresis — the trip boundary
is still exactly 0.750/1.400 V with no dead band, and a value on the wrong side of
that boundary for the full 1 s always latches. The debounce only rejects a *brief*
excursion (a spike shorter than 1 s that returns in-range on its own), which the
no-hysteresis design was never intended to protect against in the first place.

---

## Practical calibration notes

Before final commissioning:

1. Measure the actual regulated Arduino 5 V rail with a multimeter.
2. Put that measured value into `ADC_REF_V`.
3. Verify the measured divider resistances remain approximately:
   - 21.73 kOhm
   - 47.31 kOhm
4. Apply known ACT T MON voltages and compare Arduino reconstruction against a multimeter.
5. Verify:
   - 0.750 V boundary
   - 1.400 V boundary
   - fault latching
   - 10 Hz indication while out of range
   - 1 Hz indication after recovery
   - manual OFF -> ON reset requirement
6. Verify ENABLE is held LOW while:
   - Arduino boots
   - Arduino resets
   - Arduino is unpowered
   - the first fresh median window is collected after manual OFF -> ON
   - ACT T MON is already out of range when manual OFF -> ON occurs

---

## Design assumptions to verify if changing hardware

This design currently assumes:

- classic **5 V Arduino Uno/Nano**
- ATmega328P-style 10-bit ADC
- 5 V GPIO HIGH level
- PTC10K-CH ACT T MON output characteristics as above
- PTC10K-CH ENABLE thresholds as above
- external 5 V regulator shares ground with PTC supply
- bright fault LEDs are externally driven if many are fitted

If changing to a 3.3 V Arduino, modern Nano variant, ESP32, RP2040, etc., re-check:

- ADC reference / scaling
- GPIO HIGH voltage versus PTC ENABLE threshold
- GPIO current limits
- pin numbering
- analog-input maximum voltage

A 3.3 V GPIO should **not** be assumed sufficient for the PTC ENABLE input because the documented enable threshold is above 3.3 V.

---

## Summary

Final design:

```text
PTC ACT T MON
    |
 21.73k
    |
    +---- Arduino A0
    |
 47.31k
    |
   GND

Manual switch:
+5 V -- switch -- D2
                 |
                10k
                 |
                GND

PTC ENABLE:
D3 -- 1k -- PTC ENABLE
            |
           10k
            |
           GND

D4 -> green OK indication

D5-D13 + A1-A5
    |
    +-> synchronized fault indications
        preferably through external transistor-array drivers
        for 15-20 mA LEDs
```

Core behavior:

```text
GOOD + MANUAL ON
    -> ENABLE HIGH
    -> green ON
    -> fault LEDs OFF

BAD + MANUAL ON
    -> fault latched
    -> ENABLE LOW
    -> fault LEDs 10 Hz

RECOVERED AFTER FAULT
    -> ENABLE remains LOW
    -> fault LEDs 1 Hz

MANUAL OFF
    -> clears latch
    -> ENABLE LOW
    -> if still bad: fault LEDs solid ON

MANUAL ON AGAIN
    -> re-enable only if ACT T MON is currently 0.75-1.4 V
```
