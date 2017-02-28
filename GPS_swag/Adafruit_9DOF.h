#ifndef ADAFRUIT_9DOF_H
#define ADAFRUIT_9DOF_H

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

class Adafruit_IMU {
public:
  Adafruit_IMU();
  void init();
  void update();

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

private:
  Adafruit_9DOF dof;
  Adafruit_LSM303_Accel_Unified accel;
  Adafruit_LSM303_Mag_Unified mag;
};

#endif
