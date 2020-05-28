# C3PO v2

## Usage

Just turn it on! Every restart, it attempts to create a new file on the SD card in the pattern LOGXX.CSV where XX is 00 through 99

## Config settings

The config should be a file in JSON format called CONFIG.TXT in the root of the SD card.

- `readperiod`
  - Description: Delay between samples written to the SD card and reported for alarms
  - Units: microseconds
  - Default: 1000000 (one second)

- `stateperiod`
  - Description: Delay between display state cycles
  - Units: microseconds
  - Default: 3000000 (three seconds)

- `greenlimitco2max`
  - Description: Maximum allowed CO2 in PPM before alarms trigger
  - Units: PPM
  - Default: 2000

- `greenlimitpressuremin`
  - Description: Minimum allowed pressure before alarms trigger
  - Units: PSI
  - Default: 14.5

- `greenlimitpressuremax`
  - Description: Maximum allowed pressure before alarms trigger
  - Units: PSI
  - Default: 18.0

- `greenlimittemperaturemax`
  - Description: Maximum allowed temperature before alarms trigger
  - Units: Degrees Fahrenheit
  - Default: 75.0


Example config:
```
{
  "readperiod":1000000,
  "stateperiod":3000000,
  "greenlimitco2max":2000.0,
  "greenlimitpressuremin":14.5,
  "greenlimitpressuremax":18.0,
  "greenlimittemperaturemax":75.0
}
```

See [CONFIG.TXT](./CONFIG.TXT)


## Hardware

MCP23017 IO Expansion Board
- i2c MCP_ADDRESS 0x27
  - MCP_CO2_GREEN_LED_PIN 0
  - MCP_CO2_RED_LED_PIN 1
  - MCP_TEMP_GREEN_LED_PIN 2
  - MCP_TEMP_RED_LED_PIN 3
  - MCP_PRESS_GREEN_LED_PIN 4
  - MCP_PRESS_RED_LED_PIN 5
  - MCP_INPUTPIN 6

SSD1306 128x64 OLED
- i2c SSD_ADDRESS 0x3C

BME280
- i2c BME_ADDRESS 0x76 // barometric sensor

## Green level defaults

- CO2 < 2000 ppm (GREEN_LIMIT_CO2)
- Pressure +2.0 to 3.5 PSI from atmospheric
  - GREEN_LIMIT_PRESSURE_MIN = 14.5F
  - GREEN_LIMIT_PRESSURE_MAX = 18.0F
- Temperature < 75 F (GREEN_LIMIT_TEMPERATURE) = 75.0F;