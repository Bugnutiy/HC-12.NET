#include <Arduino.h>
#include <MemoryFree.h>
#include "SerialDevices.h"
// #define SERIAL_SPEED 115200

template <typename T>
void dd(const T &out)
{
  Serial.print(out);
}
template <typename T>
void ddd(const T &out)
{
  Serial.println(out);
}

SerialDevices Device;
void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial.setTimeout(100);
  Device.tick();
}
void loop()
{
  Device.tick();

  // Serial.println(freeMemory());
  // delay(100);
}

