#include <Arduino.h>
#include <SPI.h>

#include "BNO055_support.h"		
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

unsigned long lastTime = 0;

void setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
}

void loop() //This code is looped forever
{
  if ((millis() - lastTime) >= 100) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();

    bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure

    Serial.print("Time Stamp: ");				//To read out the Time Stamp
    Serial.println(lastTime);

    Serial.print("Heading(Yaw): ");				//To read out the Heading (Yaw)
    Serial.println(float(myEulerData.h) / 16.00);		//Convert to degrees

    Serial.print("Roll: ");					//To read out the Roll
    Serial.println(float(myEulerData.r) / 16.00);		//Convert to degrees

    Serial.print("Pitch: ");				//To read out the Pitch
    Serial.println(float(myEulerData.p) / 16.00);		//Convert to degrees

    Serial.println();					//Extra line to differentiate between packets
  }
}