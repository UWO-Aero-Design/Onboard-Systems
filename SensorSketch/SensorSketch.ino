#include "Arduino.h"
#include "Adafruit_9DOF.h"
#include "Quadrino_GPS.h"
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <Wire.h>

Adafruit_BMP085 bmp;
Adafruit_IMU imu;
Quadrino_GPS gps;
Servo s[7];

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void setup() {
  s[0].attach(9);
  Wire.begin();
  Serial.begin(115200);
  bmp.begin();
  imu.init();
  gps.init();
}

void loop() {
  s[0].write(60);
  imu.update();
  gps.update();

  Serial.println("--------------------------------");

  Serial.print(imu.orientation.roll);
  Serial.print("\t");
  Serial.print(imu.orientation.pitch);
  Serial.print("\t");
  Serial.println(imu.orientation.heading);
  
  if (gps.status.gps3dfix)
    Serial.println("3D");
  else if (gps.status.gps2dfix)
    Serial.println("2D");
  else
    Serial.println("No fix");
    
  Serial.print("Altitude: ");
  Serial.println(gps.alt);
  Serial.print("Latitude: ");
  Serial.println(gps.lat);
  Serial.print("Longitude: ");
  Serial.println(gps.lon);

  Serial.println("--------------------------------");
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.println("--------------------------------");
  Serial.println("--------------------------------");

  delay(100);
}
