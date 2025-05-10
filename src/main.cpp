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
#include <State.h>
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

#define TFT_CS 10
#define TFT_RST 15
#define TFT_DC 9
#define TFT_MOSI 11
#define TFT_SCLK 12

#define TFT_LED 5

#define TFT_LED_CHANNEL 0
#define TFT_LED_FREQ 5000
#define TFT_LED_RES 12

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_GFXFF

#define SPI_FREQUENCY 80000000

// cooler definitions
#define MOTOR_PIN 6
#define MOTOR_CHANNEL 1
#define MOTOR_FREQ 5000
#define MOTOR_RES 8

State state;

IEventSensor *button = new GPIOButton(PIN_BTN_DS_CONTROL);
IEventSensor *pirSensor = new PIRMotionSensor(PIN_PIR_SENSOR);

IDataSensor *co2Sensor = new CO2Sensor_MHZ19(PIN_CO2_UART_TX, PIN_CO2_UART_RX);
IDataSensor *envSensor = new TempHumPressSensor_BME280(BME280_ADDRESS);
IDataSensor *lightSensor = new LightSensor_BH1750();

// Створюємо окремий об’єкт SPI на HSPI
SPIClass *tftSPI = new SPIClass(HSPI);
// Створюємо об’єкт дисплея, передаючи посилання на tftSPI
Adafruit_ST7796S tft(tftSPI, TFT_CS, TFT_DC, TFT_RST);

int interval = 15000;
int lastRun = 0;

void setBrightness(uint16_t lightInLux, bool isOff)
{
  if (lightInLux * 4.0f >= 4095)
  {
    lightInLux = 4095;
  }
  else
  {
    lightInLux = lightInLux * 4.0f;
  }
  if (!isOff && lightInLux <= 0)
  {
    lightInLux = 3;
  }
  ledcWrite(TFT_LED_CHANNEL, lightInLux);
}

void setup()
{
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN("Starting...");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
#ifdef DEBUG_ENABLED
  I2CScan();
#endif

  ledcSetup(TFT_LED_CHANNEL, TFT_LED_FREQ, TFT_LED_RES);
  ledcAttachPin(TFT_LED, TFT_LED_CHANNEL);

  ledcSetup(MOTOR_CHANNEL, MOTOR_FREQ, MOTOR_RES);
  ledcAttachPin(MOTOR_PIN, MOTOR_CHANNEL);

  tftSPI->begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS); // MOSI and SCLK pins for HSPI
  tftSPI->setFrequency(SPI_FREQUENCY);
  tftSPI->setDataMode(SPI_MODE0);
  tftSPI->setBitOrder(MSBFIRST);

  tft.init(TFT_WIDTH, TFT_HEIGHT, 0, 0, ST7796S_BGR);
  tft.setSPISpeed(SPI_FREQUENCY);
  tft.setRotation(3);
  tft.sendCommand(ST77XX_INVOFF);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(4);
  tft.fillScreen(ST77XX_BLACK);

  button->onEvent([]() { DEBUG_PRINTLN("Button pressed CONFIRMED"); });
  button->begin();
  pirSensor->onEvent([]()
                     {
                 DEBUG_PRINTLN("Motion CONFIRMED");
                 setBrightness((uint16_t)state.getLightLevel(), false); 
                 tft.sendCommand(ST77XX_DISPON);
                 state.setDisplayOn(true); });
  pirSensor->onEventEnd([]()
                        {
     DEBUG_PRINTLN("Motion ENDED"); 
                    tft.sendCommand(ST77XX_DISPOFF);
                    setBrightness(0, true);
                    state.setDisplayOn(false); });
  pirSensor->setDebounceDelay(20);
  pirSensor->begin();
  co2Sensor->begin();
  envSensor->begin();
  lightSensor->begin();
}

void loop()
{
  long time = millis();
  button->update();
  pirSensor->update();
  // read loop
  if (lastRun == 0 || time - lastRun > (interval - 10000))
  {
    ledcWrite(MOTOR_CHANNEL, 142);
  }

  if (lastRun == 0 || time - lastRun > interval)
  {
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("Data collected:");
    state.flushData();
    
    // get data from CO2 sensor
    SensorData *data = co2Sensor->getData();
    if (CO2Data *co2Data = static_cast<CO2Data *>(data))
    {
      state.fill(co2Data);
    }
    else
    {
      DEBUG_PRINTLN("CO2 No data available");
    }
    // Get data from BME280 sensor
    SensorData *bmeData = envSensor->getData();
    if (TempHumPressureData *bmeDataPtr = static_cast<TempHumPressureData *>(bmeData))
    {
      state.fill(bmeDataPtr);
      
    }
    else
    {
      DEBUG_PRINTLN("BME280 No data available");
    }
    // Get data from BH1750 sensor
    SensorData *bhData = lightSensor->getData();
    if (LightData *bhDataPtr = static_cast<LightData *>(bhData))
    {
      state.fill(bhDataPtr);
      if (state.isDisplayOn())
      {
        setBrightness((uint16_t)state.getLightLevel(), false);
      }
    }
    else
    {
      DEBUG_PRINTLN("BH1750 No data available");
    }
    DEBUG_PRINTLN(state.toString());
      unsigned long startRender = millis();
      char buffer[128];
      tft.setCursor(5, 10);
      sprintf(buffer, "CO2: %d ppm", state.getCO2());
      tft.fillRect(5, 10, 340, 32, ST77XX_BLACK);
      tft.print(buffer);
      //
      tft.setCursor(5, 60);
      sprintf(buffer, "Temp: %.1f C", state.getTemperature());
      tft.fillRect(5, 60, 320, 32, ST77XX_BLACK);
      tft.print(buffer);
      //
      tft.setCursor(5, 110);
      sprintf(buffer, "Humidity: %.1f %%", state.getHumidity());
      tft.fillRect(5, 110, 430, 32, ST77XX_BLACK);
      tft.print(buffer);
      //
      tft.fillRect(5, 160, 470, 32, ST77XX_BLACK);
      tft.setCursor(5, 160);
      sprintf(buffer, "Pressure: %.1f hPa", state.getPressure());
      tft.print(buffer);
      //
      tft.fillRect(5, 210, 470, 36, ST77XX_BLACK);
      tft.setCursor(5, 210);
      sprintf(buffer, "Light: %.1f Lx", state.getLightLevel());
      tft.print(buffer);
      //
      int all =  millis() - time;
      tft.fillRect(5, 260, 470, 32, ST77XX_BLACK);
      tft.setCursor(5, 260);
      sprintf(buffer, "Interval: %d ms", all);
      tft.print(buffer);
      //
    DEBUG_PRINTLN("Loop time : " + String(all) + " ms");
    DEBUG_PRINTLN("Render time : " + String(millis() - startRender) + " ms");
    lastRun = time;
    ledcWrite(MOTOR_CHANNEL, 0);
  }

  delay(10); // Let CPU rest
}