#include "Arduino.h"
#include "Adafruit_9DOF.h"
#include "Quadrino_GPS.h"

Adafruit_IMU imu;
Quadrino_GPS gps;

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  imu.init();
  gps.init();
}

void loop() {
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

  delay(100);
}
