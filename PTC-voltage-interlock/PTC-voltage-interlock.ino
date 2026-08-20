// ============================================================
// PTC10K-CH Temperature Interlock
// Classic 5 V Arduino Uno / Nano (ATmega328P)
// ============================================================

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
  // Read ACT T MON
  // ==========================================================

  int adc = analogRead(PIN_TEMP);

  float vA0 =
    adc * ADC_REF_V / 1023.0;

  float vAct =
    vA0 / DIVIDER_RATIO;

  bool tempOK =
    (vAct >= V_LOW) &&
    (vAct <= V_HIGH);

  bool manualON =
    (digitalRead(PIN_MANUAL) == HIGH);


  // ==========================================================
  // MANUAL SWITCH OFF
  // ==========================================================

  if (!manualON)
  {
    // Pulling manual switch to GND resets the fault latch.
    faultLatched = false;

    // PTC always disabled while manual switch is OFF.
    digitalWrite(PIN_ENABLE, LOW);
    digitalWrite(PIN_LED_OK, LOW);

    // If temperature is still bad, fault LEDs stay solid ON.
    setFaultLEDs(!tempOK);

    return;
  }


  // ==========================================================
  // MANUAL SWITCH ON
  // ==========================================================

  // Any out-of-range event latches the fault.
  if (!tempOK) {
    faultLatched = true;
  }


  // ==========================================================
  // NORMAL ENABLED STATE
  // ==========================================================

  if (!faultLatched && tempOK)
  {
    digitalWrite(PIN_ENABLE, HIGH);
    digitalWrite(PIN_LED_OK, HIGH);
    setFaultLEDs(false);

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