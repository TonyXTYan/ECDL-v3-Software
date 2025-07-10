#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Program memory strings to save RAM
const char ads0_ch0_name[] PROGMEM = "VTEC";
const char ads0_ch1_name[] PROGMEM = "ITEC";
const char ads0_ch2_name[] PROGMEM = "TSET";
const char ads0_ch3_name[] PROGMEM = "TACT";

const char ads1_ch0_name[] PROGMEM = "IMON";
const char ads1_ch1_name[] PROGMEM = "A1C1";
const char ads1_ch2_name[] PROGMEM = "A1C2";
const char ads1_ch3_name[] PROGMEM = "A1C3";

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
const char status_no_response[] PROGMEM = "Status: No Response ";
const char status_low_voltage[] PROGMEM = "Status: Low voltage ";
const char status_overvoltage[] PROGMEM = "Status: Overvoltage ";
const char status_initializing_tec[] PROGMEM = "Status: Initializing";
const char status_connected[] PROGMEM = "38400-8N1 Connected ";
const char status_ready[] PROGMEM = "38400-8N1 Ready     ";
const char status_check_cable[] PROGMEM = "Check TEC cable     ";
const char status_disabled[] PROGMEM = "TXS0108E disabled   ";
const char status_need_voltage[] PROGMEM = "Need 2.97-3.63V     ";
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
// TXS0108E level shifter control
const int oePin = 9;        // Output Enable pin for TXS0108E
const int voltageCheckPin = A1;  // Check for 3.3V presence

// SoftwareSerial on D10 (RX), D11 (TX)
// Alternative pins for testing: D8 (RX), D12 (TX)
#define TEC_RX_PIN 10  // Change to 8 for testing
#define TEC_TX_PIN 11  // Change to 12 for testing
SoftwareSerial mySerial(TEC_RX_PIN, TEC_TX_PIN); // RX, TX

int lastPage = 0;
unsigned long lastRefresh = 0; // For 2Hz throttle
const unsigned long refreshInterval = 500; // ms

// Global variables for TEC data
float tecActualTemp = 0.0;
float tecSetTemp = 0.0;
float tecOutputPower = 0.0;
float tecPIDProportional = 4.5;
float tecPIDIntegral = 1.0;
float tecPIDDerivative = 0.5;
bool tecConnected = false;
bool tecOpenCollector = false;
bool levelShifterEnabled = false;
float supply3V3 = 0.0;
unsigned long lastTecRead = 0;
const unsigned long tecReadInterval = 3000; // Read every 3 seconds (give TEC more time)
int tecFailCount = 0; // Counter for failed TEC communications
String rawTecResponse = ""; // Store raw TEC response for display

// Global variables for ADC readings (updated by printKeyChannels)
float ads0_values[4] = {0.0, 0.0, 0.0, 0.0}; // VTEC, ITEC, TSET, TACT
float ads1_values[4] = {0.0, 0.0, 0.0, 0.0}; // IMON, A1C1, A1C2, A1C3
int16_t ads0_raw[4] = {0, 0, 0, 0};
int16_t ads1_raw[4] = {0, 0, 0, 0};
float ads0_volts[4] = {0.0, 0.0, 0.0, 0.0};
float ads1_volts[4] = {0.0, 0.0, 0.0, 0.0};

// Thermistor calculation constants
const float beta = 3375.0;        // For laser diode thermistor
// Note: TEC controller uses 10k NTC thermistor with beta=3950 (per datasheet)

#define averageLastN 10

// Pause functionality variables
bool isPaused = false;
bool lastPauseState = false;
float lastAverages_ads0[4] = {0.0, 0.0, 0.0, 0.0};
float lastAverages_ads1[4] = {0.0, 0.0, 0.0, 0.0};

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

// Custom channel names for ADS0 and ADS1 (now in PROGMEM above)

// Function to format PID values with decimal point alignment (xx.xx format)
void formatPID(float value, char* buffer) {
  // Format as xx.xx with leading zero if needed
  if (value >= 0 && value < 10) {
    dtostrf(value, 5, 2, buffer); // 5 total width, 2 decimal places: " x.xx"
    // Replace leading space with zero for alignment
    if (buffer[0] == ' ') buffer[0] = '0';
  } else {
    dtostrf(value, 5, 2, buffer); // xx.xx format
  }
  // Ensure null termination
  buffer[5] = '\0';
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

void setup() {
  // Page selector pin modes
  pinMode(pinD6, INPUT_PULLUP);
  pinMode(pinD7, INPUT_PULLUP);
  pinMode(pinD5, INPUT_PULLUP);  // Pause control pin
  
  // Level shifter control
  pinMode(oePin, OUTPUT);
  digitalWrite(oePin, LOW); // Start with level shifter disabled

  // LED output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Start serial
  Serial.begin(38400);
  // Don't start mySerial yet - wait for 3.3V detection

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print((__FlashStringHelper*)status_initializing);

  // Check for 3.3V supply
  checkSupplyVoltage();

  // Start ADS1115s with unique I2C addresses
  if (!ads0.begin(0x48)) {
    Serial.println(F("Failed to init ADS1115 #0"));
    lcd.setCursor(0, 1); lcd.print((__FlashStringHelper*)status_ads0_fail);
    while (1);
  }
  if (!ads1.begin(0x49)) {
    Serial.println(F("Failed to init ADS1115 #1"));
    lcd.setCursor(0, 2); lcd.print((__FlashStringHelper*)status_ads1_fail);
    while (1);
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

// Function to check 3.3V supply and enable/disable level shifter
void checkSupplyVoltage() {
  // Read A1 pin (0-1023 for 0-5V)
  int analogValue = analogRead(voltageCheckPin);
  supply3V3 = (analogValue * 5.0) / 1023.0;
  
  // Check if voltage is around 3.3V (allow ±10% tolerance)
  if (supply3V3 >= 2.97 && supply3V3 <= 3.63) {
    if (!levelShifterEnabled) {
      // Enable level shifter
      digitalWrite(oePin, HIGH);
      delay(5); // Reduced from 10ms to 5ms for level shifter to stabilize
      
      // Now safe to start software serial
      mySerial.begin(38400);
      levelShifterEnabled = true;
      
      Serial.print(F("3.3V detected: "));
      Serial.print(supply3V3, 2);
      Serial.println(F("V - Level shifter enabled"));
      
      // Start TEC communication at specified 38400 baud (per datasheet)
      delay(100); // Allow TEC to stabilize
    }
  } else {
    if (levelShifterEnabled) {
      // Disable level shifter
      digitalWrite(oePin, LOW);
      mySerial.end(); // Stop software serial
      levelShifterEnabled = false;
      tecConnected = false;
      
      Serial.print(F("3.3V not detected: "));
      Serial.print(supply3V3, 2);
      Serial.println(F("V - Level shifter disabled"));
    }
  }
}

void refreshPage(int page) {
  // Turn LED on for entire page refresh duration
  digitalWrite(ledPin, HIGH);
  
  // Check pause state for pages 1 and 2
  isPaused = checkPauseState();
  
  if (page == 1) {
    if (isPaused) {
      // PAUSED mode: show channel name with "PAUSED" and last average
      for (int ch = 0; ch < 4; ch++) {
        lcd.setCursor(0, ch);
        char buf[8], chName[5];
        // TSET and TACT are always positive, can use more characters for averages
        if (ch == 2 || ch == 3) {
          formatNumber(lastAverages_ads0[ch], buf, 7);  // TSET and TACT: one more decimal place
        } else {
          formatNumber(lastAverages_ads0[ch], buf, 6);  // VTEC and ITEC averages
        }
        getChannelName(chName, false, ch);
        lcd.print(chName); lcd.print(":PAUSED~");
        lcd.print(buf);
        // Use appropriate units for each channel
        if (ch == 0) {
          lcd.print("mV");  // VTEC in millivolts
        } else if (ch == 1) {
          lcd.print("mA");  // ITEC in milliamperes
        } else if (ch == 2 || ch == 3) {
          lcd.print("C");  // TSET and TACT in Celsius
        }
      }
    } else {
      // Normal mode: use global ADC values and update display
      for (int ch = 0; ch < 4; ch++) {
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
        lcd.setCursor(0, ch);
        char buf1[8], buf2[8], chName[5];
        // TSET and TACT are always positive, can use more characters for averages
        if (ch == 2 || ch == 3) {
          formatNumber(value, buf1, 6);  // Temperature current values
          formatNumber(avg, buf2, 7);    // Temperature averages with one more decimal place
        } else {
          formatNumber(value, buf1, 6);  // Current values
          formatNumber(avg, buf2, 6);    // VTEC and ITEC averages
        }
        getChannelName(chName, false, ch);
        lcd.print(chName); lcd.print(":");
        lcd.print(buf1); lcd.print("~");
        lcd.print(buf2); 
        // Use appropriate units for each channel
        if (ch == 0) {
          lcd.print("mV");  // VTEC in millivolts
        } else if (ch == 1) {
          lcd.print("mA");  // ITEC in milliamperes
        } else if (ch == 2 || ch == 3) {
          lcd.print("C");  // TSET and TACT in Celsius
        }
      }
    }
  } else if (page == 2) {
    if (isPaused) {
      // PAUSED mode: show channel name with "PAUSED" and last average
      for (int ch = 0; ch < 4; ch++) {
        lcd.setCursor(0, ch);
        char buf[8], chName[5];
        formatNumber(lastAverages_ads1[ch], buf, ch == 0 ? 6 : 6);  // IMON gets 6, A1Cx get 6 (now in mV)
        getChannelName(chName, true, ch);
        lcd.print(chName); lcd.print(":PAUSED~");
        lcd.print(buf);
        // Use appropriate units for each channel
        if (ch == 0) {
          lcd.print("mA");  // IMON in milliamperes
        } else {
          lcd.print("mV");  // A1Cx channels in millivolts
        }
      }
    } else {
      // Normal mode: use global ADC values and update display
      for (int ch = 0; ch < 4; ch++) {
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
        lcd.setCursor(0, ch);
        char buf1[7], buf2[8], chName[5];
        formatNumber(value, buf1, 6);  // All live values get one more decimal place
        formatNumber(avg, buf2, ch == 0 ? 6 : 6);  // IMON gets 6, A1Cx get 6 (now in mV)
        getChannelName(chName, true, ch);
        lcd.print(chName); lcd.print(":");
        lcd.print(buf1); lcd.print("~");
        lcd.print(buf2);
        // Use appropriate units for each channel
        if (ch == 0) {
          lcd.print("mA");  // IMON in milliamperes
        } else {
          lcd.print("mV");  // A1Cx channels in millivolts
        }
      }
    }
  } else if (page == 3) {
    // TEC Controller Page - Display raw response
    
    if (!levelShifterEnabled) {
      // Line 0: Show voltage status
      lcd.setCursor(0, 0);
      lcd.print("TEC:3.3V=");
      char buf[4];
      formatNumber(supply3V3, buf, 3);
      lcd.print(buf);
      lcd.print("V       ");  // Clear rest
      // Level shifter disabled
      lcd.setCursor(0, 1);
      if (supply3V3 < 2.97) {
        lcd.print((__FlashStringHelper*)status_low_voltage);
      } else if (supply3V3 > 3.63) {
        lcd.print((__FlashStringHelper*)status_overvoltage);
      } else {
        lcd.print((__FlashStringHelper*)status_initializing_tec);
      }
      
      lcd.setCursor(0, 2);
      lcd.print((__FlashStringHelper*)status_disabled);
      
      lcd.setCursor(0, 3);
      lcd.print((__FlashStringHelper*)status_need_voltage);
      
    } else if (tecConnected && rawTecResponse.length() > 0) {
      // TEC connected - show parsed response in compact format
      String response = rawTecResponse;
      response.trim();
      
      // Extract key values for compact display
      float tz = tecSetTemp;
      float tr = tecActualTemp; 
      float pw = tecOutputPower;
      int oc = tecOpenCollector ? 1 : 0;
      
      // Line 0: Temperatures (Tz) on left, P on right (20 chars total)
      lcd.setCursor(0, 0);
      char tzBuf[6], pBuf[6];
      formatNumber(tz, tzBuf, 5);
      formatPID(tecPIDProportional, pBuf);
      lcd.print(" Tz=");
      lcd.print(tzBuf);
      lcd.print("  P =");
      lcd.print(pBuf);
      lcd.print(" "); // Clear rest of line
      
      // Line 1: Temperature (Tr) on left, I on right (20 chars total)
      lcd.setCursor(0, 1);
      char trBuf[6], iBuf[6];
      formatNumber(tr, trBuf, 5);
      formatPID(tecPIDIntegral, iBuf);
      lcd.print(" Tr=");
      lcd.print(trBuf);
      lcd.print("  I =");
      lcd.print(iBuf);
      lcd.print(" "); // Clear rest of line
      
      // Line 2: PID D and Open Collector
      lcd.setCursor(0, 2);
      char dBuf[6];
      formatPID(tecPIDDerivative, dBuf);
      lcd.print(" OC=");
      lcd.print(oc);
      lcd.print("      D =");
      lcd.print(dBuf);
      lcd.print(" "); // Clear rest of line
      
      // Line 3: Heating/Cooling status and percentage
      lcd.setCursor(0, 3);
      int pwPercent = (int)abs(pw); // Convert to integer percentage
      
      // Show heating or cooling based on power sign
      if (pw >= 0) {
        lcd.print(" Heating");
      } else {
        lcd.print(" Cooling");
      }
      
      lcd.print("   PW=");

      // Calculate spaces needed to right-align percentage
      String percentStr = String(pwPercent) + "%";
      int usedChars = 8 + 6; // " Heating" (8) + "   PW=" (6) = 14 chars used
      int spacesNeeded = 20 - usedChars - percentStr.length() - 1;
      
      // Print spaces to right-align the percentage
      for (int i = 0; i < spacesNeeded; i++) {
        lcd.print(" ");
      }
      
      // Print the percentage
      lcd.print(percentStr);
      
    } else {
      // Level shifter enabled but no TEC response
      lcd.setCursor(0, 0);
      lcd.print("No TEC Response     ");
      
      lcd.setCursor(0, 1);
      lcd.print("Check: Cable/Power  ");
      
      lcd.setCursor(0, 2);
      lcd.print("3.3V=OK OE=1        ");
      
      lcd.setCursor(0, 3);
      if (tecFailCount > 0) {
        lcd.print("Fails:");
        lcd.print(tecFailCount);
        lcd.print("            ");
      } else {
        lcd.print("Waiting...          ");
      }
    }
  } else if (page == 0) {
    // Page 0: System status/info
    lcd.setCursor(0, 0);
    lcd.print((__FlashStringHelper*)status_system);
    
    lcd.setCursor(0, 1);
    lcd.print("ADS0: 0x48  ADS1:0x49");
    
    lcd.setCursor(0, 2); 
    lcd.print("Pages: D6="); lcd.print(digitalRead(pinD6) ? "H" : "L");
    lcd.print(" D7="); lcd.print(digitalRead(pinD7) ? "H" : "L");
    lcd.print("   ");
    
    lcd.setCursor(0, 3);
    lcd.print("Uptime: "); 
    unsigned long uptime = millis() / 1000;
    lcd.print(uptime); lcd.print("s      ");
  } else {
    lcd.setCursor(0, 1);
    lcd.print((__FlashStringHelper*)status_invalid);
  }
  
  // Turn LED off at end of page refresh
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Check supply voltage periodically (every 5 seconds)
  static unsigned long lastVoltageCheck = 0;
  if (millis() - lastVoltageCheck > 5000) {
    checkSupplyVoltage();
    lastVoltageCheck = millis();
  }

  int page = getPage();
  bool pageChanged = (page != lastPage);
  unsigned long now = millis();
  bool currentlyPaused = checkPauseState();
  bool pauseStateChanged = (currentlyPaused != lastPauseState);

  if (pageChanged) {
    lcd.clear(); // Only clear when page changes
    lastPage = page;
    lastRefresh = now; // Reset refresh timer so next regular update is in 500ms
  }

  // Only refresh if page changed, pause state changed, or if enough time has passed AND not paused
  if (pageChanged || pauseStateChanged || (!currentlyPaused && (now - lastRefresh >= refreshInterval))) {
    lastRefresh = now;
    
    // Always print key channels first for serial plotter
    printKeyChannels();
    
    // Then refresh the page display
    refreshPage(page);
  }

  // Update the last pause state
  lastPauseState = currentlyPaused;

  // Read TEC controller data (only if level shifter is enabled and not during page changes)
  if (levelShifterEnabled && !pageChanged && !pauseStateChanged) {
    readTECController();
  }
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
    ads0_raw[ch] = ads0.readADC_SingleEnded(ch);
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
    ads1_raw[ch] = ads1.readADC_SingleEnded(ch);
    ads1_volts[ch] = ads1.computeVolts(ads1_raw[ch]);
    
    if (ch == 0) { // IMON
      ads1_values[ch] = (ads1_volts[ch] / 20) * 1000;
    } else { // A1C1, A1C2, A1C3
      ads1_values[ch] = ads1_volts[ch] * 1000; // Convert to mV for display
    }
  }

  // --- Print in required order ---
  Serial.print("VTEC:"); Serial.print(ads0_values[0], 2);
  Serial.print(",ITEC:"); Serial.print(ads0_values[1], 2);
  Serial.print(",TSET:"); Serial.print(ads0_values[2], 2);
  Serial.print(",TACT:"); Serial.print(ads0_values[3], 2);
  Serial.print(",IMON:"); Serial.print(ads1_values[0], 2);
  Serial.print(",Tz:"); Serial.print(tecConnected ? tecSetTemp : 0.0, 2);
  Serial.print(",Tr:"); Serial.print(tecConnected ? tecActualTemp : 0.0, 2);
  Serial.print(",PW:"); Serial.print(tecConnected ? tecOutputPower : 0.0, 1);

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

  // --- Print A1Cx at the end ---
  Serial.print(",A1C1R:"); Serial.print(ads1_raw[1]);
  Serial.print(",A1C1V:"); Serial.print(ads1_volts[1], 4);
  Serial.print(",A1C2R:"); Serial.print(ads1_raw[2]);
  Serial.print(",A1C2V:"); Serial.print(ads1_volts[2], 4);
  Serial.print(",A1C3R:"); Serial.print(ads1_raw[3]);
  Serial.print(",A1C3V:"); Serial.print(ads1_volts[3], 4);

  Serial.println();
}

// Function to read TEC controller data  
void readTECController() {
  // Only proceed if level shifter is enabled
  if (!levelShifterEnabled) {
    tecConnected = false;
    return;
  }

  unsigned long now = millis();
  
  // Only read every tecReadInterval milliseconds
  if (now - lastTecRead < tecReadInterval) {
    return;
  }
  lastTecRead = now;
  
  // Clear any pending data
  while (mySerial.available()) {
    mySerial.read();
  }
  
  // Send 'o' command for single readout (per datasheet)
  mySerial.print("o\r\n");
  
  // Wait for response with longer timeout
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < 1000) {
    if (mySerial.available()) {
      char c = mySerial.read();
      if (c >= 32 && c <= 126) { // Printable ASCII
        response += c;
      } else if ((c == '\r' || c == '\n') && response.length() > 0) {
        break; // Complete response received
      }
    }
  }
  
  // Parse TEC response - Basic format only
  if (response.length() > 0) {
    tecConnected = true;
    tecFailCount = 0;
    rawTecResponse = response; // Store raw response
    Serial.print(F("TEC_RAW: "));
    Serial.println(response);
    
    // Reset values to defaults
    tecSetTemp = 0.0;
    tecActualTemp = 0.0;
    tecPIDProportional = 0.0;
    tecPIDIntegral = 0.0;
    tecPIDDerivative = 0.0;
    tecOutputPower = 0.0;
    tecOpenCollector = false;
    
    // C-style parsing to avoid String overhead and sscanf issues
    char buffer[response.length() + 1];
    response.toCharArray(buffer, sizeof(buffer));

    char *p;
    p = strstr(buffer, "Tz="); if(p) tecSetTemp = atof(p + 3);
    p = strstr(buffer, "P="); if(p) tecPIDProportional = atof(p + 2);
    p = strstr(buffer, "I="); if(p) tecPIDIntegral = atof(p + 2);
    p = strstr(buffer, "D="); if(p) tecPIDDerivative = atof(p + 2);
    p = strstr(buffer, "Tr="); if(p) tecActualTemp = atof(p + 3);
    p = strstr(buffer, "OC="); if(p) tecOpenCollector = (atoi(p + 3) != 0);
    
    p = strstr(buffer, "PW=");
    if (p) {
        char *pw_ptr = p + 3; // "PW=" is 3 chars
        // Handle "+ 25" case by removing space after sign
        if ((*pw_ptr == '+' || *pw_ptr == '-') && *(pw_ptr + 1) == ' ') {
            memmove(pw_ptr + 1, pw_ptr + 2, strlen(pw_ptr + 2) + 1);
        }
        tecOutputPower = atof(pw_ptr);
    }

    // Debug output for parsed values
    Serial.print(F("TEC_PARSED: Tz="));
    Serial.print(tecSetTemp, 2);
    Serial.print(F(" Tr="));
    Serial.print(tecActualTemp, 2);
    Serial.print(F(" P="));
    Serial.print(tecPIDProportional, 2);
    Serial.print(F(" I="));
    Serial.print(tecPIDIntegral, 2);
    Serial.print(F(" D="));
    Serial.print(tecPIDDerivative, 2);
    Serial.print(F(" OC="));
    Serial.print(tecOpenCollector ? 1 : 0);
    Serial.print(F(" PW="));
    Serial.println(tecOutputPower, 0);
  } else {
    // No response - mark as disconnected
    tecConnected = false;
    tecFailCount++;
    rawTecResponse = "";
    if (tecFailCount == 5) {
      Serial.print(F("TEC: No response ("));
      Serial.print(tecFailCount);
      Serial.println(F(" consecutive failures)"));
    } else if (tecFailCount % 20 == 0) {
      Serial.print(F("TEC: Still no response ("));
      Serial.print(tecFailCount);
      Serial.println(F(" failures)"));
    }
  }
}



