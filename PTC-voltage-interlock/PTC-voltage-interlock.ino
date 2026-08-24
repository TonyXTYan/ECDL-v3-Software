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
