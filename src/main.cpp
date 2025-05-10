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

//TEST
#define BLACK 0x0000
#define WHITE 0xFFFF

#include <SPI.h>
#include <TFT_eSPI.h>

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
// #define TFT_WIDTH 320
// #define TFT_HEIGHT 480

// #define TFT_CS 39
// #define TFT_RST 15
// #define TFT_DC 38
// #define TFT_MOSI 35
// #define TFT_SCLK 36

#define TFT_LED 5

#define TFT_LED_CHANNEL 0
#define TFT_LED_FREQ 5000
#define TFT_LED_RES 12

// #define LOAD_GLCD
// #define LOAD_FONT2
// #define LOAD_FONT4
// #define LOAD_GFXFF

// #define SPI_FREQUENCY 40000000

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

int interval = 60000;
int lastRun = 0;

// test start
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

int16_t h;
int16_t w;

int inc = -2;

float xx, xy, xz;
float yx, yy, yz;
float zx, zy, zz;

float fact;

int Xan, Yan;

int Xoff;
int Yoff;
int Zoff;

struct Point3d
{
  int x;
  int y;
  int z;
};

struct Point2d
{
  int x;
  int y;
};

int LinestoRender; // lines to render.
int OldLinestoRender; // lines to render just in case it changes. this makes sure the old lines all get erased.

struct Line3d
{
  Point3d p0;
  Point3d p1;
};

struct Line2d
{
  Point2d p0;
  Point2d p1;
};

Line3d Lines[20];
Line2d Render[20];
Line2d ORender[20];
/// end

void setBrightness(uint16_t lightInLux, bool isOff)
{
  if (lightInLux * 5.0f >= 4095)
  {
    lightInLux = 4095;
  }
  else
  {
    lightInLux = lightInLux * 5.0f;
  }
  if (!isOff && lightInLux <= 0)
  {
    lightInLux = 3;
  }
  ledcWrite(TFT_LED_CHANNEL, lightInLux);
}

// ------------------------------------------------ TEST ---------------------------------------------------
void RenderImage( void)
{
  // renders all the lines after erasing the old ones.
  // in here is the only code actually interfacing with the OLED. so if you use a different lib, this is where to change it.

  for (int i = 0; i < OldLinestoRender; i++ )
  {
    tft.drawLine(ORender[i].p0.x, ORender[i].p0.y, ORender[i].p1.x, ORender[i].p1.y, BLACK); // erase the old lines.
  }


  for (int i = 0; i < LinestoRender; i++ )
  {
    uint16_t color = TFT_BLUE;
    if (i < 4) color = TFT_RED;
    if (i > 7) color = TFT_GREEN;
    tft.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y, color);
  }
  OldLinestoRender = LinestoRender;
}

/***********************************************************************************************************************************/
// Sets the global vars for the 3d transform. Any points sent through "process" will be transformed using these figures.
// only needs to be called if Xan or Yan are changed.
void SetVars(void)
{
  float Xan2, Yan2, Zan2;
  float s1, s2, s3, c1, c2, c3;

  Xan2 = Xan / fact; // convert degrees to radians.
  Yan2 = Yan / fact;

  // Zan is assumed to be zero

  s1 = sin(Yan2);
  s2 = sin(Xan2);

  c1 = cos(Yan2);
  c2 = cos(Xan2);

  xx = c1;
  xy = 0;
  xz = -s1;

  yx = (s1 * s2);
  yy = c2;
  yz = (c1 * s2);

  zx = (s1 * c2);
  zy = -s2;
  zz = (c1 * c2);
}


/***********************************************************************************************************************************/
// processes x1,y1,z1 and returns rx1,ry1 transformed by the variables set in SetVars()
// fairly heavy on floating point here.
// uses a bunch of global vars. Could be rewritten with a struct but not worth the effort.
void ProcessLine(struct Line2d *ret, struct Line3d vec)
{
  float zvt1;
  int xv1, yv1, zv1;

  float zvt2;
  int xv2, yv2, zv2;

  int rx1, ry1;
  int rx2, ry2;

  int x1;
  int y1;
  int z1;

  int x2;
  int y2;
  int z2;

  int Ok;

  x1 = vec.p0.x;
  y1 = vec.p0.y;
  z1 = vec.p0.z;

  x2 = vec.p1.x;
  y2 = vec.p1.y;
  z2 = vec.p1.z;

  Ok = 0; // defaults to not OK

  xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
  yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
  zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

  zvt1 = zv1 - Zoff;

  if ( zvt1 < -5) {
    rx1 = 256 * (xv1 / zvt1) + Xoff;
    ry1 = 256 * (yv1 / zvt1) + Yoff;
    Ok = 1; // ok we are alright for point 1.
  }

  xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
  yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
  zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

  zvt2 = zv2 - Zoff;

  if ( zvt2 < -5) {
    rx2 = 256 * (xv2 / zvt2) + Xoff;
    ry2 = 256 * (yv2 / zvt2) + Yoff;
  } else
  {
    Ok = 0;
  }

  if (Ok == 1) {

    ret->p0.x = rx1;
    ret->p0.y = ry1;

    ret->p1.x = rx2;
    ret->p1.y = ry2;
  }
  // The ifs here are checks for out of bounds. needs a bit more code here to "safe" lines that will be way out of whack, so they don't get drawn and cause screen garbage.

}

/***********************************************************************************************************************************/
// line segments to draw a cube. basically p0 to p1. p1 to p2. p2 to p3 so on.
void cube(void)
{
  // Front Face.

  Lines[0].p0.x = -50;
  Lines[0].p0.y = -50;
  Lines[0].p0.z = 50;
  Lines[0].p1.x = 50;
  Lines[0].p1.y = -50;
  Lines[0].p1.z = 50;

  Lines[1].p0.x = 50;
  Lines[1].p0.y = -50;
  Lines[1].p0.z = 50;
  Lines[1].p1.x = 50;
  Lines[1].p1.y = 50;
  Lines[1].p1.z = 50;

  Lines[2].p0.x = 50;
  Lines[2].p0.y = 50;
  Lines[2].p0.z = 50;
  Lines[2].p1.x = -50;
  Lines[2].p1.y = 50;
  Lines[2].p1.z = 50;

  Lines[3].p0.x = -50;
  Lines[3].p0.y = 50;
  Lines[3].p0.z = 50;
  Lines[3].p1.x = -50;
  Lines[3].p1.y = -50;
  Lines[3].p1.z = 50;


  //back face.

  Lines[4].p0.x = -50;
  Lines[4].p0.y = -50;
  Lines[4].p0.z = -50;
  Lines[4].p1.x = 50;
  Lines[4].p1.y = -50;
  Lines[4].p1.z = -50;

  Lines[5].p0.x = 50;
  Lines[5].p0.y = -50;
  Lines[5].p0.z = -50;
  Lines[5].p1.x = 50;
  Lines[5].p1.y = 50;
  Lines[5].p1.z = -50;

  Lines[6].p0.x = 50;
  Lines[6].p0.y = 50;
  Lines[6].p0.z = -50;
  Lines[6].p1.x = -50;
  Lines[6].p1.y = 50;
  Lines[6].p1.z = -50;

  Lines[7].p0.x = -50;
  Lines[7].p0.y = 50;
  Lines[7].p0.z = -50;
  Lines[7].p1.x = -50;
  Lines[7].p1.y = -50;
  Lines[7].p1.z = -50;


  // now the 4 edge lines.

  Lines[8].p0.x = -50;
  Lines[8].p0.y = -50;
  Lines[8].p0.z = 50;
  Lines[8].p1.x = -50;
  Lines[8].p1.y = -50;
  Lines[8].p1.z = -50;

  Lines[9].p0.x = 50;
  Lines[9].p0.y = -50;
  Lines[9].p0.z = 50;
  Lines[9].p1.x = 50;
  Lines[9].p1.y = -50;
  Lines[9].p1.z = -50;

  Lines[10].p0.x = -50;
  Lines[10].p0.y = 50;
  Lines[10].p0.z = 50;
  Lines[10].p1.x = -50;
  Lines[10].p1.y = 50;
  Lines[10].p1.z = -50;

  Lines[11].p0.x = 50;
  Lines[11].p0.y = 50;
  Lines[11].p0.z = 50;
  Lines[11].p1.x = 50;
  Lines[11].p1.y = 50;
  Lines[11].p1.z = -50;

  LinestoRender = 12;
  OldLinestoRender = LinestoRender;

}

// ------------------------------------------------ end test ----

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

  button->onEvent([]() { DEBUG_PRINTLN("Button pressed CONFIRMED"); });
  button->begin();
  pirSensor->onEvent([]()
                     {
                 DEBUG_PRINTLN("Motion CONFIRMED");
                 setBrightness((uint16_t)state.getLightLevel(), false); 
                
                 state.setDisplayOn(true); });
  pirSensor->onEventEnd([]()
                        {
     DEBUG_PRINTLN("Motion ENDED"); 
                   
                    setBrightness(0, true);
                    state.setDisplayOn(false); });
  pirSensor->setDebounceDelay(20);
  pirSensor->begin();
  co2Sensor->begin();
  envSensor->begin();
  lightSensor->begin();

  SPI.begin(12, 13, 11, 10);

tft.init();

  h = tft.height();
  w = tft.width();

  tft.setRotation(1);

  tft.fillScreen(TFT_BLACK);

  cube();

  fact = 180 / 3.14159259; // conversion from degrees to radians.

  Xoff = 240; // Position the centre of the 3d conversion space into the centre of the TFT screen.
  Yoff = 160;
  Zoff = 550; // Z offset in 3D space (smaller = closer and bigger rendering)

}

void loop()
{
  // long time = millis();
  // button->update();
  // pirSensor->update();
  // // read loop
  // if (lastRun == 0 || time - lastRun > (interval - 7000))
  // {
  //   ledcWrite(MOTOR_CHANNEL, 142);
  // }

  // if (lastRun == 0 || time - lastRun > interval)
  // {
  //   DEBUG_PRINTLN("");
  //   DEBUG_PRINTLN("Data collected:");
  //   state.flushData();
    
  //   // get data from CO2 sensor
  //   SensorData *data = co2Sensor->getData();
  //   if (CO2Data *co2Data = static_cast<CO2Data *>(data))
  //   {
  //     state.fill(co2Data);
  //   }
  //   else
  //   {
  //     DEBUG_PRINTLN("CO2 No data available");
  //   }
  //   // Get data from BME280 sensor
  //   SensorData *bmeData = envSensor->getData();
  //   if (TempHumPressureData *bmeDataPtr = static_cast<TempHumPressureData *>(bmeData))
  //   {
  //     state.fill(bmeDataPtr);
  //   }
  //   else
  //   {
  //     DEBUG_PRINTLN("BME280 No data available");
  //   }
  //   // Get data from BH1750 sensor
  //   SensorData *bhData = lightSensor->getData();
  //   if (LightData *bhDataPtr = static_cast<LightData *>(bhData))
  //   {
  //     state.fill(bhDataPtr);
  //     if (state.isDisplayOn())
  //     {
  //       setBrightness((uint16_t)state.getLightLevel(), false);
  //     }
  //   }
  //   else
  //   {
  //     DEBUG_PRINTLN("BH1750 No data available");
  //   }
  //   DEBUG_PRINTLN(state.toString());
  //   DEBUG_PRINTLN("Loop time : " + String(millis() - time) + " ms");
  //   lastRun = time;
  //   ledcWrite(MOTOR_CHANNEL, 0);
  // }

  // delay(10); // Let CPU rest

  // Rotate around x and y axes in 1 degree increments
  Xan++;
  Yan++;

  Yan = Yan % 360;
  Xan = Xan % 360; // prevents overflow.

  SetVars(); //sets up the global vars to do the 3D conversion.

  // Zoom in and out on Z axis within limits
  // the cube intersects with the screen for values < 160
  Zoff += inc; 
  if (Zoff > 500) inc = -1;     // Switch to zoom in
  else if (Zoff < 160) inc = 1; // Switch to zoom out

  for (int i = 0; i < LinestoRender ; i++)
  {
    ORender[i] = Render[i]; // stores the old line segment so we can delete it later.
    ProcessLine(&Render[i], Lines[i]); // converts the 3d line segments to 2d.
  }
  RenderImage(); // go draw it!

  delay(14); // Delay to reduce loop rate (reduces flicker caused by aliasing with TFT screen refresh rate)
}