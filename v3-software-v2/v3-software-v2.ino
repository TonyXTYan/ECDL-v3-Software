// ============================================================
// ECDL v3 controller -- v2
//
// v2 = v1 (dual ADS1115 + 20x4 LCD monitoring) with the
// TEC-8A-24V-PID-HC serial bridge REMOVED and the PTC10K-CH
// temperature interlock RETROFITTED in its place.
//
// The interlock logic (rolling median, 1 s debounce, latching
// fault, manual OFF->ON reset) is carried over unchanged from
// PTC-voltage-interlock.ino. Those are deliberate safety
// choices -- no hysteresis, latch on fault, manual reset
// required -- not oversights. Do not "improve" them.
//
// See v3-software-v2-retrofit-guide.md for wiring and the
// retrofit procedure.
// ============================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

// Program memory strings to save RAM
const char ads0_ch0_name[] PROGMEM = "VTEC";
const char ads0_ch1_name[] PROGMEM = "ITEC";
const char ads0_ch2_name[] PROGMEM = "TSET";
const char ads0_ch3_name[] PROGMEM = "TACT";

// ADS1 ch2/ch3 are now the PTC10K-CH monitor taps (was A1C2/A1C3);
// ch1 stays spare (was A1C1) -- bench wiring landed ACT on ch3, not ch1.
const char ads1_ch0_name[] PROGMEM = "IMON";
const char ads1_ch1_name[] PROGMEM = "A1C1";
const char ads1_ch2_name[] PROGMEM = "SETM";
const char ads1_ch3_name[] PROGMEM = "ACTM";

// Program memory string arrays
const char* const ads0_channel_names[] PROGMEM = {
  ads0_ch0_name, ads0_ch1_name, ads0_ch2_name, ads0_ch3_name
};

const char* const ads1_channel_names[] PROGMEM = {
  ads1_ch0_name, ads1_ch1_name, ads1_ch2_name, ads1_ch3_name
};

// Status messages in program memory
const char status_initializing[] PROGMEM = "Initialising...";
const char status_ads0_fail[] PROGMEM = "ADS0 fail @0x48";
const char status_ads1_fail[] PROGMEM = "ADS1 fail @0x49";
const char status_system[] PROGMEM = "System Status       ";
const char status_invalid[] PROGMEM = "Invalid page sel.      ";

// LCD address 0x27, 20x4 display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ADS1115 instances (default constructor)
Adafruit_ADS1115 ads0;
Adafruit_ADS1115 ads1;

// Page selector pins
const int pinD6 = 6;
const int pinD7 = 7;
const int pinD5 = 5;        // Pause control pin

// Built-in LED pin
const int ledPin = 13;

int lastPage = 0;
unsigned long lastRefresh = 0; // For 2Hz throttle
const unsigned long refreshInterval = 500; // ms

// AVR Wire otherwise waits forever if the I2C bus locks. The ACT T MON
// safety decision is deliberately made with analogRead(A0), but that
// independence only helps if an I2C call eventually returns.
const unsigned long I2C_WIRE_TIMEOUT_US = 25000;
const unsigned long ADS_CONVERSION_TIMEOUT_MS = 50;
const unsigned long I2C_RECOVERY_INTERVAL_MS = 2000;
bool i2cHealthy = true;
bool lcdNeedsReinit = false;
unsigned long i2cNextRecoveryAttempt = 0;

// Global variables for ADC readings (updated by printKeyChannels)
float ads0_values[4] = {0.0, 0.0, 0.0, 0.0}; // VTEC, ITEC, TSET, TACT
float ads1_values[4] = {0.0, 0.0, 0.0, 0.0}; // IMON, A1C1, SETM, ACTM
int16_t ads0_raw[4] = {0, 0, 0, 0};
int16_t ads1_raw[4] = {0, 0, 0, 0};
float ads0_volts[4] = {0.0, 0.0, 0.0, 0.0};
float ads1_volts[4] = {0.0, 0.0, 0.0, 0.0};

// Thermistor calculation constants
const float beta = 3375.0;        // For laser diode thermistor
// Note: this is the LASER DIODE thermistor on ADS0 TSET/TACT. The
// PTC10K-CH's own 10k NTC is a different sensor with its own model
// below (PTC_BETA) -- do not conflate the two.

#define averageLastN 10

// Pause functionality variables
bool isPaused = false;
bool lastPauseState = false;
float lastAverages_ads0[4] = {0.0, 0.0, 0.0, 0.0};
float lastAverages_ads1[4] = {0.0, 0.0, 0.0, 0.0};


// ============================================================
// PTC10K-CH INTERLOCK
// ============================================================

// ---------------- Pin map ----------------
// D2/D3/D4 are committed to other hardware on this board, so the
// interlock does NOT use the standalone sketch's original pinout.
// Inputs live in the A0-A2 corner, outputs in a contiguous D9-D12
// run (D9/D10/D11 freed by removing the TXS0108E + SoftwareSerial).
const uint8_t PIN_PTC_ACT    = A0;   // ACT T MON via 21.73k/47.31k divider
const uint8_t PIN_PTC_SET    = A1;   // SET T MON via identical divider
const uint8_t PIN_PTC_MANUAL = A2;   // manual enable switch (10k pulldown)
const uint8_t PIN_PTC_ENABLE = 9;    // PTC ENABLE (1k series, 10k pulldown at PTC)
const uint8_t PIN_PTC_LED_OK = 10;   // green OK LED

// All fault LEDs are commanded identically, so they flash in sync.
// Any of these may be left physically unconnected with no firmware change.
const uint8_t PTC_FAULT_LED_PINS[] = { 11, 12 };

const uint8_t NUM_PTC_FAULT_LEDS =
  sizeof(PTC_FAULT_LED_PINS) / sizeof(PTC_FAULT_LED_PINS[0]);


// ---------------- ADC / divider ----------------

// Measure your actual regulated 5 V rail if you want better accuracy.
const float ADC_REF_V = 5.000;

// External divider (one per monitor channel):
// ACT/SET T MON -- 22k -- A0/A1 -- 47k -- GND
//
// PTC T MON outputs have ~1k output impedance, so the effective
// upper resistance is 22k + 1k.
const float R_TOP_K     = 21.70;
const float R_PTC_OUT_K = 1.00;
const float R_BOTTOM_K  = 46.65;

const float DIVIDER_RATIO =
  R_BOTTOM_K / (R_BOTTOM_K + R_TOP_K + R_PTC_OUT_K);

const float V_LOW  = 0.750;
const float V_HIGH = 1.400;

// Display-only diagnostics, not additional trip thresholds -- V_LOW/V_HIGH
// above already put both of these conditions outside the OK range, so
// ENABLE stays LOW either way. These just pick a more specific message
// than a generic FAULT for two recognizable failure signatures:
//
// An open thermistor circuit reads near ADC_REF_V at the pin, which
// after undoing the divider is ~ADC_REF_V / DIVIDER_RATIO -- around 7.4V
// with this unit's measured resistors. 6.0V is comfortably below that
// and comfortably above any real in-range or over-temperature reading.
const float NTC_DISCONNECT_V = 6.0;

// PTC10K-CH unpowered pulls both monitor outputs to ~0V (no bias
// current). A live sensor circuit won't idle this close to 0 even at
// the low end of its range.
const float PTC_NO_POWER_V = 0.05;


// ---------------- PTC sensor model -- UNCALIBRATED ----------------
// Bias current is CONFIRMED at 100 uA for this unit.
// PTC_BETA IS ASSUMED, NOT MEASURED: 3950 is the usual figure for a
// 10k NTC, but this unit's sensor has not been characterised. The
// monitor VOLTAGE is the trustworthy reading and is the only thing
// the interlock trips on; the degC readout is derived and is marked
// "~UNCAL" on page 3 until beta is fitted. The serial line prints the
// intermediate resistance so beta can be fitted against the PTC's own
// front-panel temperature. See the retrofit guide's calibration section.
const float PTC_BIAS_UA = 100.0;
const float PTC_BETA    = 3950.0;   // ASSUMED -- verify before trusting degC
const float PTC_R0_K    = 10.0;     // 10k @ 25 C


// ---------------- ADC noise rejection ----------------

// Rolling median over the last N raw ADC samples. Rejects
// single-sample spikes (ADC noise, EMI) without adding hysteresis to
// the V_LOW/V_HIGH trip thresholds themselves.
const uint8_t MEDIAN_WINDOW = 9;

// Unlike the standalone sketch (one sample per loop() pass), sampling
// here is gated to a fixed interval, because this loop() blocks: eight
// ADS1115 single-shot reads take ~70 ms and an LCD refresh tens of ms.
// serviceInterlock() is therefore also called from inside those
// blocking sections, so the median window stays a predictable
// ~45 ms and the 10 Hz fault flash never freezes mid-refresh.
const unsigned long INTERLOCK_SAMPLE_MS = 5;
unsigned long ilkLastSample = 0;

int ilkSampleBuf[MEDIAN_WINDOW];
uint8_t ilkSampleIdx = 0;
bool ilkBufferFull = false;

// Set after a SET T MON read on A1 so the next ACT sample discards one
// conversion for the ADC mux to settle.
bool ilkMuxDirty = false;

// Fault only latches after this many consecutive milliseconds of
// out-of-range readings, so a brief spike that slips past the median
// filter still can't trip the fault. A genuine, sustained excursion
// still latches within ~1 s.
const unsigned long FAULT_DEBOUNCE_MS = 1000;

bool ilkOutOfRangeActive = false;
unsigned long ilkOutOfRangeStart = 0;

// Becomes true only after a complete median window reports an
// in-range temperature while the manual switch is ON. This prevents
// startup or a manual reset from enabling an already-faulty system.
bool ilkEnableQualified = false;

// Tracks the manual switch edge so each OFF->ON cycle must collect a
// fresh median window before it can qualify ENABLE.
bool ilkManualWasON = false;

bool ilkFaultLatched = false;

// State published for the display / serial telemetry.
bool ilkManualON = false;
bool ilkEnableOut = false;
bool ilkTempOK = false;
float ptcActV_A0 = 0.0;    // interlock path (10-bit, authoritative)
float ptcSetV_A0 = 0.0;    // 10-bit
float ptcActV_ADS = 0.0;   // ADS1 ch3 (16-bit, display only)
float ptcSetV_ADS = 0.0;   // ADS1 ch2 (16-bit, display only)


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


// Set every fault LED output together.
void setFaultLEDs(bool on)
{
  for (uint8_t i = 0; i < NUM_PTC_FAULT_LEDS; i++) {
    digitalWrite(PTC_FAULT_LED_PINS[i], on ? HIGH : LOW);
  }
}


// Drive ENABLE + OK LED together so they can never disagree.
void setPtcEnable(bool on)
{
  ilkEnableOut = on;
  digitalWrite(PIN_PTC_ENABLE, on ? HIGH : LOW);
  digitalWrite(PIN_PTC_LED_OK, on ? HIGH : LOW);
}


// Fail-safe pin setup. Called FIRST in setup(), before the LCD and
// ADS1115 init, so ENABLE is driven LOW before anything can fail or
// block.
void ptcInterlockInit()
{
  pinMode(PIN_PTC_MANUAL, INPUT);

  pinMode(PIN_PTC_ENABLE, OUTPUT);
  pinMode(PIN_PTC_LED_OK, OUTPUT);

  for (uint8_t i = 0; i < NUM_PTC_FAULT_LEDS; i++) {
    pinMode(PTC_FAULT_LED_PINS[i], OUTPUT);
  }

  setPtcEnable(false);
  setFaultLEDs(false);
}


// Monitor volts -> thermistor ohms. Needs only the (confirmed) bias
// current, so this value is trustworthy even while beta is unknown.
float ptcMonVoltsToOhms(float v)
{
  return v / (PTC_BIAS_UA * 1e-6);
}


// Monitor volts -> degC via the beta equation. UNCALIBRATED: see the
// PTC sensor model block above.
float ptcMonVoltsToC(float v)
{
  float r = ptcMonVoltsToOhms(v);
  if (r <= 0.0) return -999.0;

  float invT =
    1.0 / 298.15 + (1.0 / PTC_BETA) * log(r / (PTC_R0_K * 1000.0));

  return (1.0 / invT) - 273.15;
}


// Read SET T MON on A1 (display only -- never gates ENABLE).
// Discards one conversion after the mux change, and flags the mux
// dirty so the next ACT sample does the same.
void readPtcSetMon()
{
  (void)analogRead(PIN_PTC_SET);
  int raw = analogRead(PIN_PTC_SET);
  ptcSetV_A0 = (raw * ADC_REF_V / 1023.0) / DIVIDER_RATIO;
  ilkMuxDirty = true;
}


// ------------------------------------------------------------
// The interlock state machine. Logic is carried over verbatim from
// PTC-voltage-interlock.ino; only the sampling cadence and the pin
// names differ. Safe to call as often as you like -- it self-gates to
// INTERLOCK_SAMPLE_MS.
// ------------------------------------------------------------
void serviceInterlock()
{
  unsigned long now = millis();
  if (now - ilkLastSample < INTERLOCK_SAMPLE_MS) {
    return;
  }
  ilkLastSample = now;

  // ==========================================================
  // Read ACT T MON (rolling median of last MEDIAN_WINDOW samples)
  // ==========================================================

  if (ilkMuxDirty) {
    (void)analogRead(PIN_PTC_ACT);   // discard: ADC mux settling
    ilkMuxDirty = false;
  }

  int rawAdc = analogRead(PIN_PTC_ACT);

  ilkSampleBuf[ilkSampleIdx] = rawAdc;
  ilkSampleIdx++;
  if (ilkSampleIdx >= MEDIAN_WINDOW) {
    ilkSampleIdx = 0;
    ilkBufferFull = true;
  }

  // Instantaneous (unfiltered) reading, used only to drive the fault
  // LEDs during the brief warm-up before a full median window is
  // available -- so a real fault doesn't briefly go dark, and a good
  // signal doesn't briefly flicker on, while ENABLE stays gated on
  // the debounced/qualified median value regardless.
  float vActRaw =
    (rawAdc * ADC_REF_V / 1023.0) / DIVIDER_RATIO;

  bool rawTempOK =
    (vActRaw >= V_LOW) &&
    (vActRaw <= V_HIGH);

  // Until the buffer has a full window of real samples, treat the
  // reading as not-OK (fail-safe: ENABLE stays LOW, its power-up
  // default, through the brief startup warm-up).
  bool tempOK = false;
  float vAct = vActRaw;

  if (ilkBufferFull) {
    int adc = median9(ilkSampleBuf);

    float vA0 =
      adc * ADC_REF_V / 1023.0;

    vAct =
      vA0 / DIVIDER_RATIO;

    tempOK =
      (vAct >= V_LOW) &&
      (vAct <= V_HIGH);
  }

  // Published for the display: median once available, raw during warm-up.
  ptcActV_A0 = vAct;
  ilkTempOK = tempOK;

  bool manualON =
    (digitalRead(PIN_PTC_MANUAL) == HIGH);
  ilkManualON = manualON;


  // ==========================================================
  // MANUAL SWITCH OFF
  // ==========================================================

  if (!manualON)
  {
    // Pulling manual switch to GND resets the fault latch and the
    // debounce timer, so a stale timer can't fire an instant latch
    // on the next switch-ON.
    ilkFaultLatched = false;
    ilkOutOfRangeActive = false;
    ilkEnableQualified = false;
    ilkManualWasON = false;

    // PTC always disabled while manual switch is OFF.
    setPtcEnable(false);

    // If temperature is still bad, fault LEDs stay solid ON.
    setFaultLEDs(!tempOK);

    return;
  }


  // Discard samples from before this switch-ON edge. A stale in-range
  // median must not briefly enable a system whose current input is bad.
  if (!ilkManualWasON) {
    ilkManualWasON = true;
    ilkSampleIdx = 0;
    ilkBufferFull = false;
  }


  // Do not start the debounce timer or enable the PTC until a full
  // median window is available. Fault LEDs fall back to the raw
  // instantaneous reading here so a real fault stays visibly
  // indicated (not blanked) through warm-up.
  if (!ilkBufferFull) {
    setPtcEnable(false);
    setFaultLEDs(!rawTempOK);
    return;
  }


  // ==========================================================
  // MANUAL SWITCH ON
  // ==========================================================

  // Fault only latches after FAULT_DEBOUNCE_MS of continuous
  // out-of-range readings; a single bad reading (or one the median
  // filter missed) that clears on the next pass never latches.
  if (!tempOK) {
    if (!ilkOutOfRangeActive) {
      ilkOutOfRangeActive = true;
      ilkOutOfRangeStart = now;
    } else if (now - ilkOutOfRangeStart >= FAULT_DEBOUNCE_MS) {
      ilkFaultLatched = true;
    }
  } else {
    ilkOutOfRangeActive = false;
    if (!ilkFaultLatched) {
      ilkEnableQualified = true;
    }
  }


  // ==========================================================
  // NORMAL ENABLED STATE
  // ==========================================================

  // Once an in-range reading has qualified the enabled state, gate
  // only on ilkFaultLatched so a brief out-of-range reading inside the
  // debounce window does not drop ENABLE. Before qualification (at
  // startup or after manual OFF), ENABLE remains LOW.
  if (!ilkFaultLatched)
  {
    if (ilkEnableQualified) {
      setPtcEnable(true);
      setFaultLEDs(false);
    } else {
      setPtcEnable(false);
      setFaultLEDs(!tempOK);
    }

    return;
  }


  // ==========================================================
  // FAULT-LATCHED STATE
  // ==========================================================

  setPtcEnable(false);

  if (!tempOK)
  {
    // Currently outside 0.75-1.4 V -- 10 Hz (50 ms half period)
    setFaultLEDs(((now / 50) % 2) == 0);
  }
  else
  {
    // Temperature has recovered, but fault remains latched
    // -- 1 Hz (500 ms half period)
    setFaultLEDs(((now / 500) % 2) == 0);
  }
}


// Put only the monitoring/display subsystem offline. ENABLE and the A0
// interlock state are deliberately untouched.
void markI2cOffline()
{
  if (i2cHealthy) {
    i2cHealthy = false;
    lcdNeedsReinit = true;
    i2cNextRecoveryAttempt = millis() + I2C_RECOVERY_INTERVAL_MS;
  }
}


// Record a Wire-level timeout and stop normal I2C operations until the
// bounded background recovery succeeds.
bool checkI2cTimeout()
{
  if (!Wire.getWireTimeoutFlag()) return false;

  Wire.clearWireTimeoutFlag();
  markI2cOffline();
  return true;
}


// Release the AVR TWI peripheral, clock a slave out of a possibly stuck
// byte, generate a STOP, and restart Wire. Pins are only ever driven LOW;
// the external pull-ups provide HIGH, preserving I2C open-drain behaviour.
void resetI2cBus()
{
  Wire.end();

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  for (uint8_t pulse = 0; pulse < 9 && digitalRead(SDA) == LOW; pulse++) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(5);
  }

  // STOP: SDA low-to-high while SCL is released high.
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  Wire.begin();
  Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US, true);
  Wire.clearWireTimeoutFlag();
}


bool probeI2cAddress(uint8_t address)
{
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  uint8_t result = Wire.endTransmission();
  bool timedOut = Wire.getWireTimeoutFlag();
  Wire.clearWireTimeoutFlag();
  serviceInterlock();
  return !timedOut && result == 0;
}


// Retry a failed bus without resetting the MCU or changing PTC ENABLE.
// One failed probe is bounded by I2C_WIRE_TIMEOUT_US and attempts are
// rate-limited so a hard fault cannot monopolise the safety loop.
void serviceI2cRecovery()
{
  if (i2cHealthy) {
    // LiquidCrystal_I2C::init() blocks for about a second. Re-run it only
    // while PTC ENABLE is already LOW; normal writes may still recover a
    // display that never lost power in the meantime.
    if (lcdNeedsReinit && !ilkEnableOut) {
      lcd.init();
      lcd.backlight();
      if (checkI2cTimeout()) return;
      lcdNeedsReinit = false;
      lastRefresh = millis() - refreshInterval;
    }
    return;
  }

  unsigned long now = millis();
  if ((long)(now - i2cNextRecoveryAttempt) < 0) return;
  i2cNextRecoveryAttempt = now + I2C_RECOVERY_INTERVAL_MS;

  resetI2cBus();
  serviceInterlock();

  if (!probeI2cAddress(0x27)) return;
  if (!probeI2cAddress(0x48)) return;
  if (!probeI2cAddress(0x49)) return;

  // Rebind the library objects after a device power cycle. begin() deletes
  // its previous helper before allocating a replacement, so retries do not
  // leak RAM.
  if (!ads0.begin(0x48) || checkI2cTimeout()) return;
  serviceInterlock();
  if (!ads1.begin(0x49) || checkI2cTimeout()) return;
  serviceInterlock();

  ads0.setGain(GAIN_ONE);
  ads1.setGain(GAIN_ONE);
  i2cHealthy = true;
  lastRefresh = millis() - refreshInterval;
  Serial.println(F("I2C recovered; LCD/ADS monitoring resumed"));
}


// Adafruit_ADS1X15::readADC_SingleEnded() waits indefinitely for its
// conversion-ready bit. Bound both the underlying Wire transactions and
// that higher-level polling loop so an absent or wedged ADS cannot starve
// serviceInterlock().
bool readAdsSingleEnded(Adafruit_ADS1115 &adc, uint8_t channel, int16_t &raw)
{
  static const uint16_t muxByChannel[4] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    ADS1X15_REG_CONFIG_MUX_SINGLE_3
  };

  if (!i2cHealthy || channel > 3) return false;

  Wire.clearWireTimeoutFlag();
  adc.startADCReading(muxByChannel[channel], false);
  if (checkI2cTimeout()) return false;

  unsigned long started = millis();
  while (!adc.conversionComplete()) {
    serviceInterlock();
    if (checkI2cTimeout()) return false;
    if (millis() - started >= ADS_CONVERSION_TIMEOUT_MS) {
      markI2cOffline();
      return false;
    }
  }

  if (checkI2cTimeout()) return false;
  raw = adc.getLastConversionResults();
  return !checkI2cTimeout();
}


// ============================================================
// Display helpers
// ============================================================

// Helper function to get channel name from PROGMEM
void getChannelName(char* buffer, bool isADS1, int channel) {
  const char* const* nameArray = isADS1 ? ads1_channel_names : ads0_channel_names;
  strcpy_P(buffer, (char*)pgm_read_ptr(&nameArray[channel]));
}

// Circular buffer for moving average (pages 1 and 2)
float ads0_history[4][averageLastN] = {0};
float ads1_history[4][averageLastN] = {0};
uint8_t ads0_hist_idx[4] = {0};
uint8_t ads1_hist_idx[4] = {0};
uint8_t ads0_hist_count[4] = {0};
uint8_t ads1_hist_count[4] = {0};

// Write one LCD row, space-padded to the full 20 columns and
// truncated at 20. The LCD does not wrap or clear stale characters on
// its own, so every row must be written full-width.
void lcdRow(uint8_t row, const char* s) {
  if (!i2cHealthy) return;

  lcd.setCursor(0, row);
  if (checkI2cTimeout()) return;

  uint8_t i = 0;
  for (; s[i] != '\0' && i < 20; i++) {
    lcd.print(s[i]);
    serviceInterlock();
    if (checkI2cTimeout()) return;
  }
  for (; i < 20; i++) {
    lcd.print(' ');
    serviceInterlock();
    if (checkI2cTimeout()) return;
  }
}

// Function to format number for compact display (width 2-8 characters including decimal)
void formatNumber(float value, char* buffer, int width = 6) {
  // Clamp width to valid range
  width = constrain(width, 2, 8);
  
  // Handle zero and very small values near zero
  if (abs(value) < 0.0001f) {
    strcpy(buffer, "0.");
    for (int i = 2; i < width; i++) buffer[i] = '0';
    buffer[width] = '\0';
    return;
  }
  
  // Handle negative values properly
  bool isNegative = (value < 0);
  if (isNegative) {
    value = -value; // Work with positive value
  }

  // Calculate decimal places based on magnitude to fit exactly in width
  // Account for negative sign taking up one character
  int availableWidth = isNegative ? width - 1 : width;
  int decimalPlaces;
  
  if (value >= pow(10, availableWidth - 1)) {
    decimalPlaces = 0;  // xxxxx. format for very large values
  } else {
    // Count integer digits needed - improved logic
    int intDigits;
    if (value >= 1.0) {
      intDigits = (int)log10(value) + 1;
    } else {
      intDigits = 1; // For values < 1.0, we need at least "0."
    }
    
    // Ensure we have at least 1 decimal place for small numbers
    decimalPlaces = max(1, availableWidth - intDigits - 1); // -1 for decimal point
    
    // Cap decimal places to reasonable limit
    decimalPlaces = min(decimalPlaces, 5);  // Allow up to 5 decimal places for voltage precision
  }

  // Format the number with exact width and decimal places
  dtostrf(value, availableWidth, decimalPlaces, buffer);
  
  // Add negative sign if needed
  if (isNegative) {
    // Shift everything right and add negative sign
    int len = strlen(buffer);
    memmove(buffer + 1, buffer, len + 1);
    buffer[0] = '-';
  }
  
  // Remove leading spaces for better display (dtostrf right-aligns with spaces)
  char* start = buffer;
  while (*start == ' ' && start < buffer + width) {
    start++;
  }
  if (start != buffer) {
    memmove(buffer, start, strlen(start) + 1);
  }
  
  // Ensure we don't exceed width and null terminate
  buffer[width] = '\0';
}

// Short interlock state text, shared by page 0 and page 3.
// Longest output is 11 chars, so 'out' needs at least 12 bytes.
//
// The NTC-disconnected / no-power checks run ahead of the normal
// manual/warmup/fault states: both are recognizable voltage signatures
// (see NTC_DISCONNECT_V / PTC_NO_POWER_V) worth naming specifically
// rather than folding into a generic FAULT, and they're informative
// even with the manual switch off or before the median window fills.
// Either condition already sits outside V_LOW/V_HIGH, so this is a
// display refinement only -- ENABLE was already going to stay LOW.
void ptcStateText(char* out) {
  if (ptcActV_A0 < PTC_NO_POWER_V && ptcSetV_A0 < PTC_NO_POWER_V) {
    strcpy(out, "PTC NO PWR");
  } else if (ptcActV_A0 > NTC_DISCONNECT_V || ptcSetV_A0 > NTC_DISCONNECT_V) {
    strcpy(out, "NTC DISCONN");
  } else if (!ilkManualON) {
    strcpy(out, "MANUAL OFF");
  } else if (!ilkBufferFull) {
    strcpy(out, "WARMUP");
  } else if (ilkFaultLatched) {
    strcpy(out, ilkTempOK ? "FAULT LATCH" : "FAULT 10Hz");
  } else if (ilkEnableQualified) {
    strcpy(out, "OK");
  } else {
    strcpy(out, "UNQUALIFIED");
  }
}

void setup() {
  // Interlock FIRST: pin modes and fail-safe LOW outputs before
  // anything that can block or hang.
  ptcInterlockInit();

  // Page selector pin modes
  pinMode(pinD6, INPUT_PULLUP);
  pinMode(pinD7, INPUT_PULLUP);
  pinMode(pinD5, INPUT_PULLUP);  // Pause control pin

  // LED output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Start serial
  //
  // NOTE: opening this port from a host resets the board, which drops
  // PTC ENABLE. With the manual switch left ON, the interlock then
  // re-qualifies from a fresh median window and re-enables in ~50 ms
  // with no operator action -- the documented startup behaviour, but
  // it means temperature control blips on every Serial Monitor
  // connection. See the retrofit guide.
  Serial.begin(38400);

  // Abort and reset the AVR TWI peripheral if a slave or the bus wedges.
  // This must be configured before any LCD/ADS operation.
  Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US, true);

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print((__FlashStringHelper*)status_initializing);
  if (checkI2cTimeout()) {
    Serial.println(F("Failed to init LCD/I2C bus"));
    return;
  }

  // Start ADS1115s with unique I2C addresses. A startup failure no longer
  // traps setup(); loop() keeps the A0 interlock alive and retries I2C.
  bool ads0Ready = ads0.begin(0x48);
  bool ads0TimedOut = checkI2cTimeout();
  if (!ads0Ready || ads0TimedOut) {
    Serial.println(F("Failed to init ADS1115 #0"));
    if (i2cHealthy) {
      lcd.setCursor(0, 1); lcd.print((__FlashStringHelper*)status_ads0_fail);
      checkI2cTimeout();
    }
    markI2cOffline();
    return;
  }

  bool ads1Ready = ads1.begin(0x49);
  bool ads1TimedOut = checkI2cTimeout();
  if (!ads1Ready || ads1TimedOut) {
    Serial.println(F("Failed to init ADS1115 #1"));
    if (i2cHealthy) {
      lcd.setCursor(0, 2); lcd.print((__FlashStringHelper*)status_ads1_fail);
      checkI2cTimeout();
    }
    markI2cOffline();
    return;
  }

  // Optional: Set gain (both chips)
  ads0.setGain(GAIN_ONE);
  ads1.setGain(GAIN_ONE);
  //                                                                ADS1115
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
}

void refreshPage(int page) {
  // Turn LED on for entire page refresh duration
  digitalWrite(ledPin, HIGH);
  
  // Check pause state: pages 1/2 freeze their moving averages; page 3
  // uses it to pick ADS1115 (normal) vs Nano analogRead (paused) as
  // its display source.
  isPaused = checkPauseState();
  
  if (page == 1) {
    if (isPaused) {
      // PAUSED mode: show channel name with "PAUSED" and last average
      for (int ch = 0; ch < 4; ch++) {
        serviceInterlock();
        char line[32], buf[8], chName[5];
        // TSET and TACT are always positive, can use more characters for averages
        if (ch == 2 || ch == 3) {
          formatNumber(lastAverages_ads0[ch], buf, 7);  // TSET and TACT: one more decimal place
        } else {
          formatNumber(lastAverages_ads0[ch], buf, 6);  // VTEC and ITEC averages
        }
        getChannelName(chName, false, ch);
        strcpy(line, chName); strcat(line, ":PAUSED~"); strcat(line, buf);
        // Use appropriate units for each channel
        if (ch == 0) {
          strcat(line, "mV");  // VTEC in millivolts
        } else if (ch == 1) {
          strcat(line, "mA");  // ITEC in milliamperes
        } else if (ch == 2 || ch == 3) {
          strcat(line, "C");  // TSET and TACT in Celsius
        }
        lcdRow(ch, line);
      }
    } else {
      // Normal mode: use global ADC values and update display
      for (int ch = 0; ch < 4; ch++) {
        serviceInterlock();
        float value = ads0_values[ch]; // Use pre-calculated values
        
        // Update circular buffer
        ads0_history[ch][ads0_hist_idx[ch]] = value;
        ads0_hist_idx[ch] = (ads0_hist_idx[ch] + 1) % averageLastN;
        if (ads0_hist_count[ch] < averageLastN) ads0_hist_count[ch]++;
        // Compute average
        float sum = 0.0;
        for (uint8_t i = 0; i < ads0_hist_count[ch]; i++) sum += ads0_history[ch][i];
        float avg = sum / ads0_hist_count[ch];
        
        // Store the last average for pause mode
        lastAverages_ads0[ch] = avg;
        
        // Display current / average, compact
        char line[32], buf1[8], buf2[8], chName[5];
        // TSET and TACT are always positive, can use more characters for averages
        if (ch == 2 || ch == 3) {
          formatNumber(value, buf1, 6);  // Temperature current values
          formatNumber(avg, buf2, 7);    // Temperature averages with one more decimal place
        } else {
          formatNumber(value, buf1, 6);  // Current values
          formatNumber(avg, buf2, 6);    // VTEC and ITEC averages
        }
        getChannelName(chName, false, ch);
        strcpy(line, chName); strcat(line, ":"); strcat(line, buf1);
        strcat(line, "~"); strcat(line, buf2);
        // Use appropriate units for each channel
        if (ch == 0) {
          strcat(line, "mV");  // VTEC in millivolts
        } else if (ch == 1) {
          strcat(line, "mA");  // ITEC in milliamperes
        } else if (ch == 2 || ch == 3) {
          strcat(line, "C");  // TSET and TACT in Celsius
        }
        lcdRow(ch, line);
      }
    }
  } else if (page == 2) {
    if (isPaused) {
      // PAUSED mode: show channel name with "PAUSED" and last average
      for (int ch = 0; ch < 4; ch++) {
        serviceInterlock();
        char line[32], buf[8], chName[5];
        formatNumber(lastAverages_ads1[ch], buf, 6);
        getChannelName(chName, true, ch);
        strcpy(line, chName); strcat(line, ":PAUSED~"); strcat(line, buf);
        // Use appropriate units for each channel
        if (ch == 0) {
          strcat(line, "mA");  // IMON in milliamperes
        } else {
          strcat(line, "mV");  // SETM/ACTM monitor mV, A1C1 raw mV
        }
        lcdRow(ch, line);
      }
    } else {
      // Normal mode: use global ADC values and update display
      for (int ch = 0; ch < 4; ch++) {
        serviceInterlock();
        float value = ads1_values[ch]; // Use pre-calculated values
        
        // Update circular buffer
        ads1_history[ch][ads1_hist_idx[ch]] = value;
        ads1_hist_idx[ch] = (ads1_hist_idx[ch] + 1) % averageLastN;
        if (ads1_hist_count[ch] < averageLastN) ads1_hist_count[ch]++;
        // Compute average
        float sum = 0.0;
        for (uint8_t i = 0; i < ads1_hist_count[ch]; i++) sum += ads1_history[ch][i];
        float avg = sum / ads1_hist_count[ch];
        
        // Store the last average for pause mode
        lastAverages_ads1[ch] = avg;
        
        // Display current / average, compact
        char line[32], buf1[7], buf2[8], chName[5];
        formatNumber(value, buf1, 6);
        formatNumber(avg, buf2, 6);
        getChannelName(chName, true, ch);
        strcpy(line, chName); strcat(line, ":"); strcat(line, buf1);
        strcat(line, "~"); strcat(line, buf2);
        // Use appropriate units for each channel
        if (ch == 0) {
          strcat(line, "mA");  // IMON in milliamperes
        } else {
          strcat(line, "mV");  // SETM/ACTM monitor mV, A1C1 raw mV
        }
        lcdRow(ch, line);
      }
    }
  } else if (page == 3) {
    // ============================================================
    // PTC10K-CH interlock page (replaces the deprecated TEC-8A page)
    //
    // Display source follows the D5 pause switch: normal (HIGH) shows
    // the ADS1115 (16-bit, higher-resolution) reading; paused (LOW)
    // falls back to the Nano A0/A1 analogRead (10-bit). This is a
    // display-only choice -- serviceInterlock()'s trip decision always
    // reads A0/A1 directly and never looks at this switch. Source is
    // labeled on row 2 since the two readings are no longer shown
    // side-by-side (that cross-check is still available by toggling
    // the switch and comparing the two screens).
    // ============================================================
    char line[32];
    char a[9], b[9];
    char st[14];   // ptcStateText needs >= 12 bytes

    bool useADS = !isPaused;
    float actV = useADS ? ptcActV_ADS : ptcActV_A0;
    float setV = useADS ? ptcSetV_ADS : ptcSetV_A0;

    serviceInterlock();
    formatNumber(actV, a, 6);
    formatNumber(ptcMonVoltsToC(actV), b, 7);
    strcpy(line, "ACT "); strcat(line, a); strcat(line, "V ");
    strcat(line, b); strcat(line, "C");
    lcdRow(0, line);

    serviceInterlock();
    formatNumber(setV, a, 6);
    formatNumber(ptcMonVoltsToC(setV), b, 7);
    strcpy(line, "SET "); strcat(line, a); strcat(line, "V ");
    strcat(line, b); strcat(line, "C");
    lcdRow(1, line);

    // ~UNCAL because PTC_BETA is assumed; SRC shows which reading
    // rows 0/1 are currently derived from.
    serviceInterlock();
    strcpy(line, "~UNCAL SRC:");
    strcat(line, useADS ? "ADC" : "NANO");
    lcdRow(2, line);

    serviceInterlock();
    strcpy(line, ilkEnableOut ? "EN=ON  " : "EN=OFF ");
    ptcStateText(st);
    strcat(line, st);
    lcdRow(3, line);

  } else if (page == 0) {
    // Page 0: System status/info
    char line[32], st[14];

    lcdRow(0, "System Status");
    lcdRow(1, "ADS0:0x48 ADS1:0x49");

    // Interlock summary (replaces the v1 D6/D7 pin-state debug line)
    serviceInterlock();
    formatNumber(ptcActV_A0, st, 5);
    strcpy(line, "PTC "); strcat(line, st); strcat(line, "V ");
    ptcStateText(st);
    strcat(line, st);
    lcdRow(2, line);

    unsigned long uptime = millis() / 1000;
    char uptimeText[11];
    ultoa(uptime, uptimeText, 10);
    strcpy(line, "Uptime: "); strcat(line, uptimeText); strcat(line, "s");
    lcdRow(3, line);
  } else {
    lcdRow(1, "Invalid page sel.");
  }
  
  // Turn LED off at end of page refresh
  digitalWrite(ledPin, LOW);
}

void loop() {
  serviceInterlock();
  serviceI2cRecovery();

  int page = getPage();
  bool pageChanged = (page != lastPage);
  unsigned long now = millis();
  bool currentlyPaused = checkPauseState();
  bool pauseStateChanged = (currentlyPaused != lastPauseState);

  if (pageChanged) {
    if (i2cHealthy) {
      lcd.clear(); // Only clear when page changes
      checkI2cTimeout();
    }
    lastPage = page;
    lastRefresh = now; // Reset refresh timer so next regular update is in 500ms
  }

  // D5 pauses only the two moving-average pages. On system/interlock
  // pages, both the display and serial telemetry continue while it is LOW.
  bool pauseAppliesToPage = currentlyPaused && (page == 1 || page == 2);
  if (pageChanged || pauseStateChanged || (!pauseAppliesToPage && (now - lastRefresh >= refreshInterval))) {
    lastRefresh = now;

    // SET T MON (display only) -- read on the display cadence
    readPtcSetMon();

    // Always print key channels first for serial plotter
    printKeyChannels();

    // Then refresh the page display
    if (i2cHealthy) refreshPage(page);
  }

  // Update the last pause state
  lastPauseState = currentlyPaused;
}

int getPage() {
  bool d6 = !digitalRead(pinD6);
  bool d7 = !digitalRead(pinD7);

  // 3 pages: from three-position switch
  if (!d6 && d7)  return 1;
  if (!d6 && !d7) return 2; 
  if (d6 && !d7)  return 3;
  return 0;
}

// Function to check if pages 1 and 2 should be paused
bool checkPauseState() {
  return !digitalRead(pinD5);  // LOW (GND) = paused, HIGH (5V) = normal
}

// Function to read all ADC channels and print for serial plotter
void printKeyChannels() {
  // Read all ADS0 channels and store globally
  for (int ch = 0; ch < 4; ch++) {
    serviceInterlock();   // ~8 ms per single-shot read: keep the interlock fed
    int16_t raw;
    if (!readAdsSingleEnded(ads0, ch, raw)) break;
    ads0_raw[ch] = raw;
    ads0_volts[ch] = ads0.computeVolts(ads0_raw[ch]);
    
    if (ch == 0) { // VTEC
      ads0_values[ch] = 2.0 * (ads0_volts[ch] - 2.5) * 1000.0;
    } else if (ch == 1) { // ITEC
      ads0_values[ch] = 2.0 * (ads0_volts[ch] - 2.5) * 1000.0;
    } else if (ch == 2 || ch == 3) { // TSET, TACT
      float v_offset = ads0_volts[ch] - 1.65;
      float numerator = 4.096 - v_offset;
      float denominator = 4.096 + v_offset;
      if (numerator > 0 && denominator > 0) {
        float temp_kelvin = 1.0 / ((1.0/beta) * log(numerator/denominator) + 1.0/(25.0+273.15));
        ads0_values[ch] = temp_kelvin - 273.15;
      } else {
        ads0_values[ch] = -999.0;
      }
    }
  }

  // Read all ADS1 channels and store globally
  for (int ch = 0; ch < 4; ch++) {
    serviceInterlock();
    int16_t raw;
    if (!readAdsSingleEnded(ads1, ch, raw)) break;
    ads1_raw[ch] = raw;
    ads1_volts[ch] = ads1.computeVolts(ads1_raw[ch]);
    
    if (ch == 0) { // IMON
      ads1_values[ch] = (ads1_volts[ch] / 20) * 1000;
    } else if (ch == 2 || ch == 3) {
      // SETM/ACTM: these channels tap the PTC monitor DIVIDER NODE
      // (the raw 0-6 V monitor would exceed the +/-4.096 V GAIN_ONE
      // range), so undo the divider to report monitor mV.
      ads1_values[ch] = (ads1_volts[ch] / DIVIDER_RATIO) * 1000;
    } else { // A1C1 spare
      ads1_values[ch] = ads1_volts[ch] * 1000; // Convert to mV for display
    }
  }

  // 16-bit view of the PTC monitors -- display/logging only. The
  // interlock never trips on these: they arrive over bounded I2C calls,
  // while the independent A0 path remains authoritative.
  ptcActV_ADS = ads1_values[3] / 1000.0;
  ptcSetV_ADS = ads1_values[2] / 1000.0;

  // --- Print in required order ---
  Serial.print("VTEC:"); Serial.print(ads0_values[0], 2);
  Serial.print(",ITEC:"); Serial.print(ads0_values[1], 2);
  Serial.print(",TSET:"); Serial.print(ads0_values[2], 2);
  Serial.print(",TACT:"); Serial.print(ads0_values[3], 2);
  Serial.print(",IMON:"); Serial.print(ads1_values[0], 2);

  // --- PTC10K-CH interlock ---
  Serial.print(",PACTV:"); Serial.print(ptcActV_A0, 4);
  Serial.print(",PACTADS:"); Serial.print(ptcActV_ADS, 4);
  Serial.print(",PSETV:"); Serial.print(ptcSetV_A0, 4);
  Serial.print(",PSETADS:"); Serial.print(ptcSetV_ADS, 4);
  Serial.print(",PACTR:"); Serial.print(ptcMonVoltsToOhms(ptcActV_A0), 0);
  Serial.print(",PACTC:"); Serial.print(ptcMonVoltsToC(ptcActV_A0), 2);
  Serial.print(",PSETC:"); Serial.print(ptcMonVoltsToC(ptcSetV_A0), 2);
  Serial.print(",PEN:"); Serial.print(ilkEnableOut ? 1 : 0);
  Serial.print(",PFLT:"); Serial.print(ilkFaultLatched ? 1 : 0);
  Serial.print(",PI2C:"); Serial.print(i2cHealthy ? 1 : 0);

  // --- Print raw/volts for key channels ---
  Serial.print(",VTECR:"); Serial.print(ads0_raw[0]);
  Serial.print(",VTECV:"); Serial.print(ads0_volts[0], 4);
  Serial.print(",ITECR:"); Serial.print(ads0_raw[1]);
  Serial.print(",ITECV:"); Serial.print(ads0_volts[1], 4);
  Serial.print(",TSETR:"); Serial.print(ads0_raw[2]);
  Serial.print(",TSETV:"); Serial.print(ads0_volts[2], 4);
  Serial.print(",TACTR:"); Serial.print(ads0_raw[3]);
  Serial.print(",TACTV:"); Serial.print(ads0_volts[3], 4);
  Serial.print(",IMONR:"); Serial.print(ads1_raw[0]);
  Serial.print(",IMONV:"); Serial.print(ads1_volts[0], 4);

  // --- Print monitor taps / spare at the end ---
  Serial.print(",ACTMR:"); Serial.print(ads1_raw[3]);
  Serial.print(",ACTMV:"); Serial.print(ads1_volts[3], 4);
  Serial.print(",SETMR:"); Serial.print(ads1_raw[2]);
  Serial.print(",SETMV:"); Serial.print(ads1_volts[2], 4);
  Serial.print(",A1C1R:"); Serial.print(ads1_raw[1]);
  Serial.print(",A1C1V:"); Serial.print(ads1_volts[1], 4);

  Serial.println();
}
