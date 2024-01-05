#include <Arduino.h>
#include "Dbg.h"
#include "SerialDevices.h"

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

