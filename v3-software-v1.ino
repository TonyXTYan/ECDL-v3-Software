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
float tecTempMin = 0.0;
float tecTempMax = 60.0;
bool tecConnected = false;
bool tecPowerOn = false;
bool tecOpenCollector = false;
bool levelShifterEnabled = false;
float supply3V3 = 0.0;
unsigned long lastTecRead = 0;
const unsigned long tecReadInterval = 2000; // Read every 2 seconds
int tecFailCount = 0; // Counter for failed TEC communications

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

// Function to format number for compact display (width 2-8 characters including decimal)
void formatNumber(float value, char* buffer, int width = 6) {
  // Clamp width to valid range
  width = constrain(width, 2, 8);
  
  // Handle zero
  if (value == 0.0f) {
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
    // Count integer digits needed
    int intDigits = (value >= 1.0) ? (int)log10(value) + 1 : 1;
    decimalPlaces = max(0, availableWidth - intDigits - 1); // -1 for decimal point
  }

  // Format the number
  dtostrf(value, availableWidth, decimalPlaces, buffer);
  
  // Remove leading spaces and ensure proper formatting
  char temp[16];
  strcpy(temp, buffer);
  
  // Find first non-space character
  int start = 0;
  while (temp[start] == ' ' && start < strlen(temp)) start++;
  
  // Copy without leading spaces
  strcpy(buffer, &temp[start]);
  
  // Add negative sign if needed
  if (isNegative) {
    int len = strlen(buffer);
    memmove(buffer + 1, buffer, len + 1);
    buffer[0] = '-';
  }
  
  // Ensure we don't exceed width
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
  Serial.begin(9600);
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
  ads0.setGain(GAIN_TWOTHIRDS);
  ads1.setGain(GAIN_TWOTHIRDS);
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
        formatNumber(lastAverages_ads0[ch], buf, 7);
        getChannelName(chName, false, ch);
        lcd.print(chName); lcd.print(":PAUSED~");
        lcd.print(buf);
        // Use appropriate units for each channel
        if (ch == 1) {
          lcd.print("A");  // ITEC in amperes
        } else if (ch == 2 || ch == 3) {
          lcd.print("C");  // TSET and TACT in Celsius
        } else {
          lcd.print("V");  // VTEC in volts
        }
      }
    } else {
      // Normal mode: read ADC and update display
      for (int ch = 0; ch < 4; ch++) {
        float value = ads0.computeVolts(ads0.readADC_SingleEnded(ch));
        
        // Apply VTEC formula for channel 0: VTEC = 2*(V-2.5)
        if (ch == 0) {
          value = 2.0 * (value - 2.5);
        }
        // Apply ITEC formula for channel 1: ITEC = 2*(V-2.5)
        if (ch == 1) {
          value = 2.0 * (value - 2.5);
        }
        // Apply temperature formula for channels 2 and 3: TSET and TACT
        // T = (1/beta * ln((4.096-(V-1.65))/(4.096+(V-1.65))) + 1/(25+273.15))^-1
        if (ch == 2 || ch == 3) {
          float v_offset = value - 1.65;
          float numerator = 4.096 - v_offset;
          float denominator = 4.096 + v_offset;
          
          // Check for valid range to avoid log of negative numbers
          if (numerator > 0 && denominator > 0) {
            float temp_kelvin = 1.0 / ((1.0/beta) * log(numerator/denominator) + 1.0/(25.0+273.15));
            value = temp_kelvin - 273.15; // Convert to Celsius
          } else {
            value = -999.0; // Error value for invalid readings
          }
        }
        
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
        char buf1[7], buf2[8], chName[5];
        formatNumber(value, buf1);
        formatNumber(avg, buf2, 7);
        getChannelName(chName, false, ch);
        lcd.print(chName); lcd.print(":");
        lcd.print(buf1); lcd.print("~");
        lcd.print(buf2); 
        // Use appropriate units for each channel
        if (ch == 1) {
          lcd.print("A");  // ITEC in amperes
        } else if (ch == 2 || ch == 3) {
          lcd.print("C");  // TSET and TACT in Celsius
        } else {
          lcd.print("V");  // VTEC in volts
        }
      }
    }
  } else if (page == 2) {
    if (isPaused) {
      // PAUSED mode: show channel name with "PAUSED" and last average
      for (int ch = 0; ch < 4; ch++) {
        lcd.setCursor(0, ch);
        char buf[8], chName[5];
        formatNumber(lastAverages_ads1[ch], buf, ch == 0 ? 6 : 7);  // IMON gets 6 chars, others get 7
        getChannelName(chName, true, ch);
        lcd.print(chName); lcd.print(":PAUSED~");
        lcd.print(buf);
        // Use appropriate units for each channel
        if (ch == 0) {
          lcd.print("mA");  // IMON in milliamperes
        } else {
          lcd.print("V");  // Other channels in volts
        }
      }
    } else {
      // Normal mode: read ADC and update display
      for (int ch = 0; ch < 4; ch++) {
        float value = ads1.computeVolts(ads1.readADC_SingleEnded(ch));
        
        // Apply IMON formula for channel 0: IMON = (V/20)*1000 (in mA)
        if (ch == 0) {
          if (value > 0.0) {
            value = (value / 20) * 1000;
          } else {
            value = -999.0; // Error value for invalid readings
          }
        }
        
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
        formatNumber(value, buf1);
        formatNumber(avg, buf2, ch == 0 ? 6 : 7);  // IMON gets 6 chars, others get 7
        getChannelName(chName, true, ch);
        lcd.print(chName); lcd.print(":");
        lcd.print(buf1); lcd.print("~");
        lcd.print(buf2);
        // Use appropriate units for each channel
        if (ch == 0) {
          lcd.print("mA");  // IMON in milliamperes
        } else {
          lcd.print("V");  // Other channels in volts
        }
      }
    }
  } else if (page == 3) {
    // TEC Controller Page - Compact 20x4 layout
    char buf[12];  // Increased buffer size for safety
    
    // Line 0: Header with voltage and OE status (must fit 20 chars)
    lcd.setCursor(0, 0);
    lcd.print("TEC:3.3V=");
    if (levelShifterEnabled) {
      lcd.print("OK");
    } else {
      formatNumber(supply3V3, buf, 4);
      lcd.print(buf);
    }
    lcd.print(" OE=");
    lcd.print(levelShifterEnabled ? "1" : "0");
    lcd.print("  ");  // Fill remaining space
    
    if (!levelShifterEnabled) {
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
      
    } else if (tecConnected) {
      // TEC connected - show detailed info
      
      // Line 1: Temperatures with open collector status
      lcd.setCursor(0, 1);
      lcd.print("Act:");
      formatNumber(tecActualTemp, buf, 4);
      lcd.print(buf);
      lcd.print(" Set:");
      formatNumber(tecSetTemp, buf, 4);
      lcd.print(buf);
      lcd.print(tecOpenCollector ? "!C" : "C");  // '!' if OC active
      
      // Line 2: Power output with direction
      lcd.setCursor(0, 2);
      if (tecPowerOn) {
        if (tecOutputPower > 0) {
          lcd.print("Heat:");
          formatNumber(tecOutputPower, buf, 5);
          lcd.print(buf); lcd.print("%        ");
        } else if (tecOutputPower < 0) {
          lcd.print("Cool:");
          formatNumber(-tecOutputPower, buf, 5);
          lcd.print(buf); lcd.print("%        ");
        } else {
          lcd.print("Idle: 0.0%          ");
        }
      } else {
        lcd.print("Power: OFF          ");
      }
      
      // Line 3: PID parameters or limits
      lcd.setCursor(0, 3);
      lcd.print("P:");
      formatNumber(tecPIDProportional, buf, 3);
      lcd.print(buf);
      lcd.print(" I:");
      formatNumber(tecPIDIntegral, buf, 3);
      lcd.print(buf);
      lcd.print(" D:");
      formatNumber(tecPIDDerivative, buf, 3);
      lcd.print(buf);
      lcd.print("   ");
      
    } else {
      // Level shifter enabled but no TEC response
      lcd.setCursor(0, 1);
      lcd.print((__FlashStringHelper*)status_no_response);
      
      lcd.setCursor(0, 2);
      lcd.print((__FlashStringHelper*)status_check_cable);
      
      lcd.setCursor(0, 3);
      lcd.print((__FlashStringHelper*)status_ready);
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
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < 500) {
    if (mySerial.available()) {
      char c = mySerial.read();
      if (c >= 32 && c <= 126) { // Printable ASCII
        response += c;
      } else if ((c == '\r' || c == '\n') && response.length() > 0) {
        break; // Complete response received
      }
    }
  }
  
  // Parse TEC response according to datasheet format:
  // <Tsetpoint P I D Tmin Tmax Tmeasured OC PWM>
  if (response.length() > 0) {
    tecConnected = true;
    tecFailCount = 0;
    
    // Remove angle brackets if present
    String cleanResponse = response;
    cleanResponse.trim();
    if (cleanResponse.startsWith("<") && cleanResponse.endsWith(">")) {
      cleanResponse = cleanResponse.substring(1, cleanResponse.length() - 1);
    }
    
    // Parse 9 space-separated values
    float values[9];
    int valueCount = 0;
    int startPos = 0;
    
    for (int i = 0; i < 9 && valueCount < 9; i++) {
      int spacePos = cleanResponse.indexOf(' ', startPos);
      String valueStr;
      
      if (spacePos == -1) { // Last value
        valueStr = cleanResponse.substring(startPos);
      } else {
        valueStr = cleanResponse.substring(startPos, spacePos);
        startPos = spacePos + 1;
      }
      
      if (valueStr.length() > 0) {
        values[valueCount++] = valueStr.toFloat();
      }
    }
    
    // Assign values according to datasheet protocol
    if (valueCount >= 9) {
      tecSetTemp = values[0];           // Tsetpoint
      tecPIDProportional = values[1];   // P
      tecPIDIntegral = values[2];       // I  
      tecPIDDerivative = values[3];     // D
      tecTempMin = values[4];           // Tmin
      tecTempMax = values[5];           // Tmax
      tecActualTemp = values[6];        // Tmeasured
      tecOpenCollector = (values[7] > 0.5); // OC status
      tecOutputPower = values[8];       // PWM percentage
      tecPowerOn = (abs(tecOutputPower) > 0.1);
      
      Serial.print(F("TEC: Set="));
      Serial.print(tecSetTemp, 1);
      Serial.print(F("°C Act="));
      Serial.print(tecActualTemp, 1);
      Serial.print(F("°C PWM="));
      Serial.print(tecOutputPower, 1);
      Serial.println(F("%"));
    } else {
      Serial.println(F("TEC: Incomplete response"));
    }
  } else {
    // No response - mark as disconnected
    tecConnected = false;
    tecFailCount++;
    
         if (tecFailCount >= 5) {
       Serial.print(F("TEC: No response ("));
       Serial.print(tecFailCount);
       Serial.println(F(" consecutive failures)"));
       
       // Run diagnostics every 10 failures
       if (tecFailCount % 10 == 0) {
         diagnoseTECConnection();
       }
     }
  }
}

// Function to configure TEC with full parameters (per datasheet)
// Command format: <T setpoint P I D T min T max>
void configureTEC(float setpoint, float p, float i, float d, float tmin, float tmax) {
  if (!levelShifterEnabled) {
    Serial.println(F("TEC: Cannot send command - level shifter disabled"));
    return;
  }
  
  // Validate parameters according to datasheet
  p = constrain(p, 0.0, 20.0);     // P coefficient range 0.0-20.0
  i = constrain(i, 0.0, 20.0);     // I coefficient range 0.0-20.0  
  d = constrain(d, 0.0, 20.0);     // D coefficient range 0.0-20.0
  tmin = constrain(tmin, -100.0, 100.0); // Temperature range
  tmax = constrain(tmax, -100.0, 100.0); // Temperature range
  
  // Format command string with proper precision
  String cmd = "<" + String(setpoint, 1) + " " + 
               String(p, 1) + " " + String(i, 1) + " " + String(d, 1) + " " +
               String(tmin, 1) + " " + String(tmax, 1) + ">\r\n";
  
  mySerial.print(cmd);
  Serial.print(F("TEC Config: "));
  Serial.println(cmd);
}

// Function to set TEC temperature using current PID parameters
void setTECTemperature(float temperature) {
  configureTEC(temperature, tecPIDProportional, tecPIDIntegral, 
               tecPIDDerivative, tecTempMin, tecTempMax);
}

// Function to turn TEC power ON (per datasheet: 'A' command)
void turnTECOn() {
  if (!levelShifterEnabled) {
    Serial.println(F("TEC: Cannot send command - level shifter disabled"));
    return;
  }
  
  mySerial.print("A\r\n");
  Serial.println(F("TEC: Power ON"));
}

// Function to turn TEC power OFF (per datasheet: 'a' command)  
void turnTECOff() {
  if (!levelShifterEnabled) {
    Serial.println(F("TEC: Cannot send command - level shifter disabled"));
    return;
  }
  
  mySerial.print("a\r\n");
  Serial.println(F("TEC: Power OFF"));
}

// Function to start/stop cyclic readout mode
void setTECCyclicMode(bool enable) {
  if (!levelShifterEnabled) {
    Serial.println(F("TEC: Cannot send command - level shifter disabled"));
    return;
  }
  
  if (enable) {
    mySerial.print("R\r\n");  // Turn ON cyclic print
    Serial.println(F("TEC: Cyclic mode ON"));
  } else {
    mySerial.print("r\r\n");  // Turn OFF cyclic print  
    Serial.println(F("TEC: Cyclic mode OFF"));
  }
}



// Function to diagnose TEC connection issues
void diagnoseTECConnection() {
  Serial.println(F("\n=== TEC DIAGNOSTICS ==="));
  
  // Check level shifter enable pin
  Serial.print(F("Level Shifter OE (D9): "));
  Serial.println(digitalRead(oePin) ? "ENABLED" : "DISABLED");
  
  // Check 3.3V supply voltage
  int analogValue = analogRead(voltageCheckPin);
  float voltage = (analogValue * 5.0) / 1023.0;
  Serial.print(F("3.3V Supply (A1): "));
  Serial.print(voltage, 2);
  Serial.print(F("V - "));
  if (voltage >= 2.97 && voltage <= 3.63) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("OUT OF RANGE"));
  }
  
  // Check TEC communication pins
  Serial.print(F("TEC RX Pin (D")); Serial.print(TEC_RX_PIN); Serial.print(F("): "));
  pinMode(TEC_RX_PIN, INPUT_PULLUP);
  delay(10);
  Serial.println(digitalRead(TEC_RX_PIN) ? "HIGH" : "LOW");
  
  Serial.print(F("TEC TX Pin (D")); Serial.print(TEC_TX_PIN); Serial.print(F("): "));
  Serial.println(digitalRead(TEC_TX_PIN) ? "HIGH" : "LOW");
  
  // Check TEC connection status
  Serial.print(F("TEC Connected: "));
  Serial.println(tecConnected ? "YES" : "NO");
  
  Serial.print(F("Consecutive Failures: "));
  Serial.println(tecFailCount);
  
  // Restore normal operation
  if (levelShifterEnabled) {
    digitalWrite(oePin, HIGH);
    if (!mySerial) {
      mySerial.begin(38400);
    }
  } else {
    digitalWrite(oePin, LOW);
  }
  
  Serial.println(F("=== END DIAGNOSTICS ===\n"));
}
