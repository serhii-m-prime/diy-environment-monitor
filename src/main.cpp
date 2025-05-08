#include <Arduino.h>
#include <Wire.h>
#include <Debug.h>
#include <IEventSensor.h>
#include <IDataSensor.h>
#include <GPIOButton.h>
#include <PIRMotionSensor.h>
#include <CO2Sensor_MHZ19.h>
#include <TempHumPressSensor_BME280.h>

// Pin definitions
#define PIN_BUILT_IN_LED 2
#define PIN_BTN_DS_CONTROL 19
#define PIN_PIR_SENSOR 35

// All I2C sensors pins
#define PIN_I2C_SDA 41
#define PIN_I2C_SCL 40
#define BME280_ADDRESS 0x76

// CO2 sensor UART pins
#define PIN_CO2_UART_TX 18
#define PIN_CO2_UART_RX 17

// Display SPI pins
#define PIN_DISPLAY_CS 1
#define PIN_DISPLAY_RESET 1
#define PIN_DISPLAY_DC_RS 1
#define PIN_DISPLAY_SDI 1
#define PIN_DISPLAY_SCK 1

IEventSensor* button = new GPIOButton(PIN_BTN_DS_CONTROL);
IEventSensor* pir = new PIRMotionSensor(PIN_PIR_SENSOR);

IDataSensor* co2Sensor = new CO2Sensor_MHZ19(PIN_CO2_UART_TX, PIN_CO2_UART_RX);
IDataSensor* bme280 = new TempHumPressSensor_BME280(BME280_ADDRESS);

int interval = 20000;
int lastRun = 0;

/**
 * @brief Scan I2C bus for connected devices
 */
void I2CScan() {
  DEBUG_PRINTLN("Scanning I2C bus...");

  byte count = 0;

  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      DEBUG_PRINT("Found I2C device at 0x");
      DEBUG_PRINTLN(address, HEX);
      ++count;
    } else if (error == 4) {
      DEBUG_PRINT("Unknown error at 0x");
      DEBUG_PRINTLN(address, HEX);
    }
  }

  if (count == 0) {
    DEBUG_PRINTLN("No I2C devices found.");
  } else {
    DEBUG_PRINTF("Scan complete. %d device(s) found.\n", count);
  }
}

void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN("Starting...");
  delay(100);
  // Initailize I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  #ifdef DEBUG_ENABLED
    I2CScan();
  #endif
  // button
  button->onEvent([]() {
    DEBUG_PRINTLN("Button pressed CONFIRMED");
  });
  button->begin();
  // pir
  pir->onEvent([]() {
    DEBUG_PRINTLN("Motion CONFIRMED");
  });
  pir->onEventEnd([]() {
    DEBUG_PRINTLN("Motion ENDED");
  });
  pir->setDebounceDelay(20);
  pir->begin();
  // CO2
  co2Sensor->begin();
  // BME280
  bme280->begin();
}

void loop() {
  button->update();
  pir->update();
  //read loop
  long time = millis();
  if (lastRun == 0 || time - lastRun > interval) {
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("Data collected:");
    // get data from CO2 sensor
    SensorData* data = co2Sensor->getData();
    if (auto* co2Data = static_cast<CO2Data*>(data)) {
      DEBUG_PRINT("CO2: ");
      DEBUG_PRINT(co2Data->ppm);
      DEBUG_PRINT(" ppm, ");
      DEBUG_PRINT("Temperature: ");
      DEBUG_PRINT(co2Data->temperature);
      DEBUG_PRINT(" °C");
      DEBUG_PRINT(", Time: ");
      DEBUG_PRINTLN(co2Sensor->getLastUpdateTime());
    } else {
      DEBUG_PRINTLN("CO2 No data available");
    }
    // Get data from BME280 sensor
    SensorData* bmeData = bme280->getData();
    if (auto* bmeDataPtr = static_cast<TempHumPressureData*>(bmeData)) {
      DEBUG_PRINT("Temperature: ");
      DEBUG_PRINT(bmeDataPtr->temperature);
      DEBUG_PRINT(" °C, ");
      DEBUG_PRINT("Humidity: ");
      DEBUG_PRINT(bmeDataPtr->humidity);
      DEBUG_PRINT(" %, ");
      DEBUG_PRINT("Pressure: ");
      DEBUG_PRINT(bmeDataPtr->pressure);
      DEBUG_PRINT(" hPa");
      DEBUG_PRINT(", Time: ");
      DEBUG_PRINTLN(bme280->getLastUpdateTime());
    } else {
      DEBUG_PRINTLN("BME280 No data available");
    }
    DEBUG_PRINTLN("");
    lastRun = time;
  }

  delay(10); // Let CPU rest
}