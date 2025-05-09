#include <Arduino.h>
#include <Wire.h>
#include <Debug.h>
#include <IEventSensor.h>
#include <IDataSensor.h>
#include <GPIOButton.h>
#include <PIRMotionSensor.h>
#include <CO2Sensor_MHZ19.h>
#include <TempHumPressSensor_BME280.h>
#include <LightSensor_BH1750.h>

#include <SPI.h>
#include <Adafruit_ST7796S.h>

// Pin definitions
#define PIN_BUILT_IN_LED 2
#define PIN_BTN_DS_CONTROL 19
#define PIN_PIR_SENSOR 4

// All I2C sensors pins
#define PIN_I2C_SDA 41
#define PIN_I2C_SCL 40
#define BME280_ADDRESS 0x76
#define BH1750_ADDRESS 0x23

// CO2 sensor UART pins
#define PIN_CO2_UART_TX 18
#define PIN_CO2_UART_RX 17

// Display settings
// #define ST7796_DRIVER
#define TFT_WIDTH 320
#define TFT_HEIGHT 480

#define TFT_CS 39
#define TFT_RST 15
#define TFT_DC 38
#define TFT_MOSI 35
#define TFT_SCLK 36

#define TFT_LED 5

#define TFT_LED_CHANNEL 0
#define TFT_LED_FREQ 5000
#define TFT_LED_RES 12

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_GFXFF

#define SPI_FREQUENCY 40000000

// cooler definitions
#define MOTOR_PIN 6
#define MOTOR_CHANNEL 1
#define MOTOR_FREQ 5000
#define MOTOR_RES 8

IEventSensor *button = new GPIOButton(PIN_BTN_DS_CONTROL);
IEventSensor *pir = new PIRMotionSensor(PIN_PIR_SENSOR);

IDataSensor *co2Sensor = new CO2Sensor_MHZ19(PIN_CO2_UART_TX, PIN_CO2_UART_RX);
IDataSensor *bme280 = new TempHumPressSensor_BME280(BME280_ADDRESS);
IDataSensor *bh1750 = new LightSensor_BH1750();

Adafruit_ST7796S tft = Adafruit_ST7796S(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

int interval = 60000;
int lastRun = 0;
int displayOn = 0;
float currentLightLevel = 0.0f;

/**
 * @brief Scan I2C bus for connected devices
 */
void I2CScan()
{
  DEBUG_PRINTLN("Scanning I2C bus...");

  byte count = 0;

  for (byte address = 1; address < 127; ++address)
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      DEBUG_PRINT("Found I2C device at 0x");
      DEBUG_PRINTLN(address, HEX);
      ++count;
    }
    else if (error == 4)
    {
      DEBUG_PRINT("Unknown error at 0x");
      DEBUG_PRINTLN(address, HEX);
    }
  }

  if (count == 0)
  {
    DEBUG_PRINTLN("No I2C devices found.");
  }
  else
  {
    DEBUG_PRINTF("Scan complete. %d device(s) found.\n", count);
  }
}

void setBrightness(uint16_t lightInLux)
{
  if (lightInLux * 5.0f >= 4095)
  {
    lightInLux = 4095;
  }
  else
  {
    lightInLux = lightInLux * 5.0f;
  }
  ledcWrite(TFT_LED_CHANNEL, lightInLux);
}

void setup()
{
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN("Starting...");
  delay(100);
  // Initailize I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
#ifdef DEBUG_ENABLED
  I2CScan();
#endif

  ledcSetup(TFT_LED_CHANNEL, TFT_LED_FREQ, TFT_LED_RES);
  ledcAttachPin(TFT_LED, TFT_LED_CHANNEL);

  ledcSetup(MOTOR_CHANNEL, MOTOR_FREQ, MOTOR_RES);
  ledcAttachPin(MOTOR_PIN, MOTOR_CHANNEL);

  tft.init(TFT_WIDTH, TFT_HEIGHT, 0, 0, ST7796S_BGR);
  tft.setSPISpeed(SPI_FREQUENCY);
  tft.setRotation(3);
  tft.sendCommand(ST77XX_INVOFF);

  // button
  button->onEvent([]()
                  { DEBUG_PRINTLN("Button pressed CONFIRMED"); });
  button->begin();
  // pir
  pir->onEvent([]()
               {
                 DEBUG_PRINTLN("Motion CONFIRMED");
                 setBrightness((uint16_t)currentLightLevel); 
                 tft.sendCommand(ST77XX_DISPON);
                 displayOn = 1; });
  pir->onEventEnd([]()
                  { DEBUG_PRINTLN("Motion ENDED"); 
                    tft.sendCommand(ST77XX_DISPOFF);
                    setBrightness(0);
                    displayOn = 0; });
  pir->setDebounceDelay(20);
  pir->begin();
  // CO2
  co2Sensor->begin();
  // BME280
  bme280->begin();
  // BH1750
  bh1750->begin();
}

void loop()
{
  long time = millis();
  button->update();
  pir->update();
  // read loop
  if (lastRun == 0 || time - lastRun > (interval - 7000))
  {
    ledcWrite(MOTOR_CHANNEL, 142);
  }

  if (lastRun == 0 || time - lastRun > interval)
  {
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("Data collected:");
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(4);

    // get data from CO2 sensor
    SensorData *data = co2Sensor->getData();
    if (auto *co2Data = static_cast<CO2Data *>(data))
    {
      DEBUG_PRINT("[MHZ19C] CO2: ");
      DEBUG_PRINT(co2Data->ppm);
      DEBUG_PRINT(" ppm, ");
      DEBUG_PRINT("Temperature: ");
      DEBUG_PRINT(co2Data->temperature);
      DEBUG_PRINT(" °C");
      DEBUG_PRINT(", Time: ");
      DEBUG_PRINTLN(co2Sensor->getLastUpdateTime());
      tft.setCursor(5, 5);
      tft.print("CO2: ");
      tft.print(co2Data->ppm);
      tft.print(" ppm");
      tft.setCursor(5, 45);
      tft.print("Temp: ");
      tft.print(co2Data->temperature);
      tft.print(" C");
    }
    else
    {
      DEBUG_PRINTLN("CO2 No data available");
    }
    // Get data from BME280 sensor
    SensorData *bmeData = bme280->getData();
    if (auto *bmeDataPtr = static_cast<TempHumPressureData *>(bmeData))
    {
      DEBUG_PRINT("[BME280]Temperature: ");
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
      tft.setCursor(5, 85);
      tft.print("Temp: ");
      tft.print(bmeDataPtr->temperature);
      tft.print(" C");
      tft.setCursor(5, 125);
      tft.print("Humidity: ");
      tft.print(bmeDataPtr->humidity);
      tft.print(" %");
      tft.setCursor(5, 165);
      tft.print("Pressure: ");
      char buffer[10];
      sprintf(buffer, "%.1f", bmeDataPtr->pressure);
      tft.print(buffer);
      tft.print(" hPa");
    }
    else
    {
      DEBUG_PRINTLN("BME280 No data available");
    }
    // Get data from BH1750 sensor
    SensorData *bhData = bh1750->getData();
    if (auto *bhDataPtr = static_cast<LightData *>(bhData))
    {
      DEBUG_PRINT("[BH1750]Light: ");
      DEBUG_PRINT(bhDataPtr->lux);
      DEBUG_PRINT(" lx, ");
      DEBUG_PRINT("Time: ");
      DEBUG_PRINTLN(bh1750->getLastUpdateTime());
      tft.setCursor(5, 205);
      tft.print("Light: ");
      tft.print(bhDataPtr->lux);
      tft.print(" Lx");
      tft.setCursor(5, 245);
      tft.print("Iteration: ");
      tft.print(millis() - time);
      tft.print(" ms");
      tft.setCursor(5, 285);
      currentLightLevel = bhDataPtr->lux;
      if (displayOn)
      {
        setBrightness((uint16_t)currentLightLevel);
      }
    }
    else
    {
      DEBUG_PRINTLN("BH1750 No data available");
    }
    DEBUG_PRINTLN("");
    lastRun = time;
    ledcWrite(MOTOR_CHANNEL, 0);
  }

  delay(10); // Let CPU rest
}