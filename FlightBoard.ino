/* 
UWO AERO DESIGN PLANE FOR SAE AERO DESIGN EAST COMPETITION.
ALL CODE FOR THIS PROJECT WAS DEVELOPED BY THE TEAM MEMBERS
OF THE UNIVERSITY OF WESTERN ONTARIO AERODESIGN TEAM.

THIS CODE IS TARGETING SIMPLE WAYPOINT NAVIGATION OF AN RC
AIRCRAFT USING A GPS AND ONBOARD SENSORS TO RELAY TELEMETRY
TO A GROUND STATION.

Creator: Duncan Iglesias

2013-2016
*/

#pragma once
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>

// Adadfruit Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_GPS.h>
#include <Adafruit_PWMServoDriver.h>

#include "Plane.h"

Plane plane = Plane();


#define DEBUG
#define OUTPUT_READABLE_ACCELGYRO

float count1 = millis();
float count2 = millis();

void setup()
{  
  // preflight checks and system initiation
  plane.setup();
  delay(1000);
  
} 

void loop()
{
 
  if (millis()-count1 > 200) {
    plane.getLocation();
    count1 = millis();
  }

  if (millis()-count2 > 500) {
    plane.getGroundStation();
    plane.updateOnboardStates();
    plane.sendCommunicationsToGroundStation();
    count2 = millis();
  }
}
