#pragma once
#include <Wire.h>

#ifdef DEBUG_ENABLED
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif


/**
 * @brief Scan I2C bus for connected devices
 */
inline void I2CScan()
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