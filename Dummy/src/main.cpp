#include <Arduino.h>
#include <SPI.h>

#include "BNO055_support.h"		
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_euler myEulerData;

unsigned long lastTime = 0;

void setup()
{
  Wire.begin();
  BNO_Init(&myBNO); 
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1);
  Serial.begin(115200);
}

void loop()
{
  if ((millis() - lastTime) >= 100)
  {
    lastTime = millis();
    bno055_read_euler_hrp(&myEulerData);
    Serial.print("Heading(Yaw): ");
    Serial.println(float(myEulerData.h) / 16.00);
  }
}