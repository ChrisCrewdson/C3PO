#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1306.h>
#include <gfxfont.h>
#include <RTClib.h>
#include <SD.h>
#include <splash.h>
#include <Wire.h>

#define MCP_ADDRESS 0x27
#define SSD_ADDRESS 0x3C
#define BME_ADDRESS 0x76

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define LOGO_HEIGHT   64
#define LOGO_WIDTH    64

#define MCP_CO2_GREEN_LED_PIN 0
#define MCP_CO2_RED_LED_PIN 1
#define MCP_TEMP_GREEN_LED_PIN 2
#define MCP_TEMP_RED_LED_PIN 3
#define MCP_PRESS_GREEN_LED_PIN 4
#define MCP_PRESS_RED_LED_PIN 5
#define MCP_INPUTPIN 6

#define CSV_HEADER F("datetime,co2,temperature,pressure,humidity")

#define CARD_CHIP_SELECT 4

const unsigned long READ_PERIOD = 1000000; // one second
const unsigned long STATE_PERIOD = 3000000; // three seconds

// green levels
// co2 < 2000 ppm
const float GREEN_LIMIT_CO2 = 2000.0F;
// pressure 2.0-3.5 PSI
const float GREEN_LIMIT_PRESSURE_MIN = 14.5F;
const float GREEN_LIMIT_PRESSURE_MAX = 18.0F;
// temperature < 75 F
const float GREEN_LIMIT_TEMPERATURE = 75.0F;
// const float GREEN_LIMIT_TEMPERATURE = 95.0F;

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

File logfile;

int co2 = 400; // todo: remove in favor of sensor

bool manual = false;
int displayState = 0;
int alarm = 0;
int buttonState = LOW;     // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 75;   // the debounce time; increase if the output flickers

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, SSD_ADDRESS);
  drawlogo();

  while(!Serial && millis() < 3000) {
    //wait for USB serial to connect or 3 seconds to elapse
  }
  Serial.begin(9600);
  Serial.println(F("startup"));

  bool sdCardReady = SD.begin(CARD_CHIP_SELECT);
  if (!sdCardReady) {
    Serial.println(F("Card init. failed!"));
    error(2);
  } else {
    Serial.println(F("SD card ready"));
  }

  // todo: check for and load config for alarm limits

  char filename[15];
  strcpy(filename, "/LOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      Serial.print(F("will try to use "));
      Serial.println(filename);
      break;
    }
  }
  logfile = SD.open(filename, FILE_WRITE);
  if(!logfile) {
    Serial.print(F("Couldnt create ")); 
    Serial.println(filename);
    error(3);
  }
  
  logfile.println(CSV_HEADER);
  logfile.flush();
  Serial.println(CSV_HEADER);
  
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

  delay(1000); // show logo for a second
}

void loop() {
  if (displayState > 3) displayState = 0;
  if (alarm > 3) alarm = 0;

  refreshButtonState();

  static unsigned long lastRead;
  if (micros() - lastRead >= READ_PERIOD) {
    lastRead += READ_PERIOD;

    float temperatureC = bme.readTemperature(); // C
    float temperature = temperatureC * 9.0F / 5.0F + 32.0F;
    float pressureHpa = bme.readPressure(); // hPa -> .0001450 PSI
    float pressure = pressureHpa * 0.00015F; // PSI
    float humidity = bme.readHumidity();

    writeToSD(co2, temperature, pressure, humidity);

    alarm = updateAlarm(co2, temperature, pressure);
    updateLEDs(alarm);
    updateDisplay(alarm > 0 ? alarm : displayState, co2, temperature, pressure);
  }

  static unsigned long lastStateChange;
  if (micros() - lastStateChange >= STATE_PERIOD) {
    // todo: remove debugging output
    Serial.print(F("alarm: ")); Serial.print(alarm);
    Serial.print(F(" buttonState: ")); Serial.print(buttonState);
    Serial.print(F(" manual: ")); Serial.print(manual);
    Serial.print(F(" displayState: ")); Serial.println(displayState);

    lastStateChange += STATE_PERIOD;
    if (!manual) {
      displayState++;
      if (displayState > 3) displayState = 1; // cycling skips summary
    }
  }
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

int updateAlarm(const int co2, const float temperature, const float pressure) {
  if (co2 > GREEN_LIMIT_CO2) {
    return 1;
  }
  if (temperature > GREEN_LIMIT_TEMPERATURE) {
    return 2;
  }
  if (pressure < GREEN_LIMIT_PRESSURE_MIN || pressure > GREEN_LIMIT_PRESSURE_MAX) {
    return 3;
  }
  return 0;
}

void updateLEDs(const int alarm) {
  switch (alarm) {
    case 1: // CO2
      mcp.digitalWrite(MCP_CO2_RED_LED_PIN, HIGH);
      mcp.digitalWrite(MCP_CO2_GREEN_LED_PIN, LOW);
      break;
    case 2: // TEMP
      mcp.digitalWrite(MCP_TEMP_RED_LED_PIN, HIGH);
      mcp.digitalWrite(MCP_TEMP_GREEN_LED_PIN, LOW);
      break;
    case 3: // PRESS
      mcp.digitalWrite(MCP_PRESS_RED_LED_PIN, HIGH);
      mcp.digitalWrite(MCP_PRESS_GREEN_LED_PIN, LOW);
      break;
    case 0:
    default:
      mcp.digitalWrite(MCP_CO2_GREEN_LED_PIN, HIGH);
      mcp.digitalWrite(MCP_TEMP_GREEN_LED_PIN, HIGH);
      mcp.digitalWrite(MCP_PRESS_GREEN_LED_PIN, HIGH);

      mcp.digitalWrite(MCP_CO2_RED_LED_PIN, LOW);
      mcp.digitalWrite(MCP_TEMP_RED_LED_PIN, LOW);
      mcp.digitalWrite(MCP_PRESS_RED_LED_PIN, LOW);
      break;
  }
}

void updateDisplay(const int displayState, const int co2, const float temperature, const float pressure) {
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
    case 3:
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
    case 0:
    default:
      displaySummary(co2, temperature, pressure);
  }
  
}

void displaySummary(const int co2, const float temperature, const float pressure) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.setTextSize(2);

  display.print(F("CO2: "));
  display.println(co2);
  display.print(F("Pres: "));
  display.println(pressure, 1);
  display.print(F("Temp: "));
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
    // delay, so take it as the actual current state:

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

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}
