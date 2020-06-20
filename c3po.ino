#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <gfxfont.h>
#include <RTClib.h>
#include <SD.h>
#include <splash.h>
#include <Wire.h>

#define __AVR__
// CO2 sensor
// from lib: https://github.com/SandboxElectronics/UART_Bridge
#include <SC16IS750.h>
// from lib: https://github.com/SandboxElectronics/NDIRZ16
#include <NDIRZ16.h>

#define MCP_ADDRESS 0x27 // io breakout
#define SSD_ADDRESS 0x3C // oled display
#define BME_ADDRESS 0x76 // barometric sensor
#define MHZ_ADDRESS 0x9A // co2 sensor

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define LOGO_HEIGHT   64
#define LOGO_WIDTH    64

#define MCP_CO2_RED_LED_PIN 0
#define MCP_CO2_GREEN_LED_PIN 1
#define MCP_PRESS_RED_LED_PIN 2
#define MCP_PRESS_GREEN_LED_PIN 3
#define MCP_TEMP_RED_LED_PIN 4
#define MCP_TEMP_GREEN_LED_PIN 5

#define MCP_INPUTPIN 7

#define CSV_HEADER F("datetime,co2,temperature,pressure,humidity")

#define ERR_CARD_INIT_FAILED F("Card init. failed!")

#define CARD_CHIP_SELECT 4

struct Config {
  long readperiod;
  long stateperiod;
  float greenlimitco2max;
  float greenlimitpressuremin;
  float greenlimitpressuremax;
  float greenlimittemperaturemax;
};

const char *filename = "/CONFIG.TXT";
Config config;

const unsigned long READ_PERIOD_DEFAULT = 1000000; // one second
const unsigned long STATE_PERIOD_DEFAULT = 3000000; // three seconds

// green levels
// co2 < 4000 ppm
const float GREEN_LIMIT_CO2_MAX_DEFAULT = 4000.0F;
// pressure -2.0 to +3.5 PSI from sea level
const float GREEN_LIMIT_PRESSURE_MIN_DEFAULT = -2.0F;
const float GREEN_LIMIT_PRESSURE_MAX_DEFAULT = 3.5F;
// temperature < 90 F
const float GREEN_LIMIT_TEMPERATURE_MAX_DEFAULT = 90.0F;

const unsigned char psfLogo [] PROGMEM = { // 64x64px
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x1f, 0xdf, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x9f, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x1f, 0x9f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x8f, 0xc0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x3f, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x0f, 0xc0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x3f, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x07, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x7e, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x07, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x01, 0xf8, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0xf8, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x01, 0xf8, 0x00, 0x00, 
  0x00, 0x00, 0x03, 0xf0, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0xfc, 0x00, 0x00, 
  0x00, 0x00, 0x03, 0xf0, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x7c, 0x00, 0x00, 
  0x00, 0x00, 0x07, 0xe0, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x7e, 0x00, 0x00, 
  0x00, 0x00, 0x07, 0xe0, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x3f, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0xc0, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x1f, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0x80, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x1f, 0x80, 0x00, 
  0x00, 0x00, 0x1f, 0x80, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x0f, 0xc0, 0x00, 
  0x00, 0x00, 0x3f, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x0f, 0xc0, 0x00, 
  0x00, 0x00, 0x3e, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x07, 0xe0, 0x00, 
  0x00, 0x00, 0x7e, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x03, 0xe0, 0x00, 
  0x00, 0x00, 0xfc, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x03, 0xf0, 0x00, 
  0x00, 0x00, 0xf8, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x01, 0xf0, 0x00
};

Adafruit_MCP23017 mcp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BME280 bme;
RTC_DS3231 rtc;

// CO2: MH-Z16 and UART adapter
SC16IS750 co2i2cUart = SC16IS750(SC16IS750_PROTOCOL_I2C,SC16IS750_ADDRESS_BB);
NDIRZ16 co2Sensor = NDIRZ16(&co2i2cUart);

File logfile;

bool manual = false;
int displayState = 0;
int alarms[] = {false, false, false}; // co2, press, temp
int buttonState = LOW;     // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 75;   // the debounce time; increase if the output flickers

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, SSD_ADDRESS);
  drawlogo();

  while(!Serial && millis() < 3000) {
    // wait for USB serial to connect or 3 seconds to elapse
  }
  Serial.begin(9600);
  displayMessage(F("startup"));

  bool sdCardReady = SD.begin(CARD_CHIP_SELECT);
  if (!sdCardReady) {
    displayMessage(ERR_CARD_INIT_FAILED);
    fatalError(2);
  } else {
    displayMessage(F("SD card ready"));
  }

  // check for and load config for alarm limits and sampling rate
  loadConfiguration(filename, config);

  char filename[15];
  strcpy(filename, "/LOG00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      Serial.print(F("Logging to "));
      Serial.println(filename);
      break;
    }
  }
  logfile = SD.open(filename, FILE_WRITE);
  if(!logfile) {
    displayMessage(F("Could not create log file!"));
    Serial.println(filename);
    fatalError(3);
  }

  logfile.println(CSV_HEADER);
  logfile.flush();
  Serial.println(CSV_HEADER);

  // CO2 sensor setup
  co2i2cUart.begin(9600);
  if (co2i2cUart.ping()) {
    for (uint8_t i = 0; i < 10; i++) {
      displayMessage("Wait " + String(10 - i) + " seconds for CO2 sensor initialization...");
      delay(1000);
    }
  } else {
    displayMessage(F("CO2 sensor not found."));
  }
  power(1);

  bme.begin(BME_ADDRESS);

  mcp.begin(MCP_ADDRESS);
  mcp.pinMode(MCP_CO2_GREEN_LED_PIN, OUTPUT);
  mcp.pinMode(MCP_CO2_RED_LED_PIN, OUTPUT);
  mcp.pinMode(MCP_TEMP_GREEN_LED_PIN, OUTPUT);
  mcp.pinMode(MCP_TEMP_RED_LED_PIN, OUTPUT);
  mcp.pinMode(MCP_PRESS_GREEN_LED_PIN, OUTPUT);
  mcp.pinMode(MCP_PRESS_RED_LED_PIN, OUTPUT);
  mcp.pinMode(MCP_INPUTPIN, INPUT);
  mcp.pullUp(MCP_INPUTPIN, HIGH);  // turn on a 100K pullup internally
}

void loop() {
  if (displayState > 3) displayState = 0;

  refreshButtonState();

  static unsigned long lastRead;
  float temperatureC;
  float temperature;
  float pressureHpa;
  float pressure;
  float humidity;
  int co2 = 0;

  if (micros() - lastRead >= config.readperiod) {
    lastRead += config.readperiod;

    temperatureC = bme.readTemperature(); // C
    temperature = temperatureC * 9.0F / 5.0F + 32.0F;
    pressureHpa = bme.readPressure(); // hPa -> .0001450 PSI
    pressure = pressureHpa * 0.00015F - 14.6959F; // PSI delta from 1 ATM
    humidity = bme.readHumidity();

    if (co2Sensor.measure()) {
      co2 = co2Sensor.ppm;
    } else {
      displayMessage(F("CO2 sensor communication error"));
    }

    writeToSD(co2, temperature, pressure, humidity);

    updateAlarms(alarms, co2, temperature, pressure);
    updateLEDs(alarms);
    updateDisplay(alarms, displayState, co2, temperature, pressure);
  }

  static unsigned long lastStateChange;
  if (micros() - lastStateChange >= config.stateperiod) {
    lastStateChange += config.stateperiod;
    if (!manual) {
      displayState++;
      if (displayState > 3) displayState = 1; // cycling skips summary
    }
    updateDisplay(alarms, displayState, co2, temperature, pressure);
  }
}

// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  File file = SD.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  DeserializationError deserialError = deserializeJson(doc, file);
  if (deserialError) {
    displayMessage(F("Failed to read CONFIG, using defaults"));
    Serial.println(deserialError.c_str());
  } else {
    displayMessage(F("Read CONFIG from SD"));
  }

  // Copy values from the JsonDocument to the Config
  config.readperiod = doc["readperiod"] | READ_PERIOD_DEFAULT;
  config.stateperiod = doc["stateperiod"] | STATE_PERIOD_DEFAULT;
  config.greenlimitco2max = doc["greenlimitco2max"] | GREEN_LIMIT_CO2_MAX_DEFAULT;
  config.greenlimitpressuremin = doc["greenlimitpressuremin"] | GREEN_LIMIT_PRESSURE_MIN_DEFAULT;
  config.greenlimitpressuremax = doc["greenlimitpressuremax"] | GREEN_LIMIT_PRESSURE_MAX_DEFAULT;
  config.greenlimittemperaturemax = doc["greenlimittemperaturemax"] | GREEN_LIMIT_TEMPERATURE_MAX_DEFAULT;

  file.close();

  serializeJsonPretty(doc, Serial);
  Serial.println();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("read period: "); display.println(config.readperiod);
  display.print("state period: "); display.println(config.stateperiod);
  display.print("co2 max: "); display.println(config.greenlimitco2max);
  display.print("pressure min: "); display.println(config.greenlimitpressuremin);
  display.print("pressure max: "); display.println(config.greenlimitpressuremax);
  display.print("temperature max: "); display.println(config.greenlimittemperaturemax);
  display.display();
  delay(1000);
}

void drawlogo(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    psfLogo, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
}

String getISOTimeNow() {
  DateTime now = rtc.now();
  // build an ISO 8601 datetime string
  char buf1[20];
  sprintf(buf1, "%04d-%02d-%02dT%02d:%02d:%02dZ", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  return String(buf1);
}

void showTime(const String now) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println(now);
  display.display();
}

void updateAlarms(int alarms[], const int co2, const float temperature, const float pressure) {
  if (co2 > config.greenlimitco2max) {
    alarms[0] = true;
  } else {
    alarms[0] = false;
  }

  if (pressure < config.greenlimitpressuremin || pressure > config.greenlimitpressuremax) {
    alarms[1] = true;
  } else {
    alarms[1] = false;
  }

  if (temperature > config.greenlimittemperaturemax) {
    alarms[2] = true;
  } else {
    alarms[2] = false;
  }
}

void updateLEDs(int alarms[]) {
  if (alarms[0]) {
    mcp.digitalWrite(MCP_CO2_RED_LED_PIN, HIGH);
    mcp.digitalWrite(MCP_CO2_GREEN_LED_PIN, LOW);
  } else {
    mcp.digitalWrite(MCP_CO2_GREEN_LED_PIN, HIGH);
    mcp.digitalWrite(MCP_CO2_RED_LED_PIN, LOW);
  }

  if (alarms[1]) {
    mcp.digitalWrite(MCP_PRESS_RED_LED_PIN, HIGH);
    mcp.digitalWrite(MCP_PRESS_GREEN_LED_PIN, LOW);
  } else {
    mcp.digitalWrite(MCP_PRESS_GREEN_LED_PIN, HIGH);
    mcp.digitalWrite(MCP_PRESS_RED_LED_PIN, LOW);
  }

  if (alarms[2]) {
    mcp.digitalWrite(MCP_TEMP_RED_LED_PIN, HIGH);
    mcp.digitalWrite(MCP_TEMP_GREEN_LED_PIN, LOW);
  } else {
    mcp.digitalWrite(MCP_TEMP_GREEN_LED_PIN, HIGH);
    mcp.digitalWrite(MCP_TEMP_RED_LED_PIN, LOW);
  }
}

void updateDisplay(int alarms[], const int displayState, const int co2, const float temperature, const float pressure) {
  switch (displayState) {
    case 1:
      // co2
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.setTextSize(3);
      display.println(F("CO2"));
      display.setTextSize(4);
      display.println(co2);
      display.display();
      break;
    case 2:
      // pressure
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.setTextSize(3);
      display.println(F("Pres"));
      display.setTextSize(4);
      display.println(pressure, 2);
      display.display();
      break;
    case 3:
      // temperature
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.setTextSize(3);
      display.println(F("Temp"));
      display.setTextSize(4);
      display.println(temperature, 1);
      display.display();
      break;
    case 0:
    default:
      displaySummary(co2, temperature, pressure);
  }
  
}

void displaySummary(const int co2, const float temperature, const float pressure) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.setTextSize(1);

  display.print(F("CO2(max ")); display.print(config.greenlimitco2max); display.println(F(" ppm):"));
  display.println(co2);

  display.print(F("Pres(min ")); display.print(config.greenlimitpressuremin); 
  display.print(F(",max ")); display.print(config.greenlimitpressuremax); display.println(F(" PSI):"));
  display.println(pressure, 1);

  display.print(F("Temp(max ")); display.print(config.greenlimittemperaturemax); display.println(F(" F):"));
  display.println(temperature, 1);

  display.display();
}

void writeToSD(const int co2, const float temperature, const float pressure, const float humidity) {
  String dataString = "";
  dataString += getISOTimeNow();
  dataString += ",";
  dataString += String(co2);
  dataString += ",";
  dataString += String(temperature);
  dataString += ",";
  dataString += String(pressure);
  dataString += ",";
  dataString += String(humidity);
  logfile.println(dataString);
  logfile.flush();

  Serial.println(dataString);
}

void refreshButtonState(void) {
  // read the state of the switch into a local variable:
  int reading = mcp.digitalRead(MCP_INPUTPIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        Serial.println(F("pressed"));
        manual = !manual;
        displayState = 0;
      } else {
        Serial.println(F("released"));
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}

void displayMessage(const String data) {
  Serial.println(data);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println(data);
  display.display();
  delay(1000);
}

void fatalError(const uint8_t errorNumber) {
  while(1) {
    for (uint8_t i = 0; i < errorNumber; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (uint8_t i = errno; i < 10; i++) {
      delay(200);
    }
  }
}

// Power control function for NDIR CO2 sensor. 1=ON, 0=OFF
void power (uint8_t state) {
  co2i2cUart.pinMode(0, INPUT); // set up the power control pin

  if (state) {
    co2i2cUart.pinMode(0, INPUT); // turn on the power of MH-Z16
  } else {
    co2i2cUart.pinMode(0, OUTPUT);
    co2i2cUart.digitalWrite(0, 0); // turn off the power of MH-Z16
  }
}
