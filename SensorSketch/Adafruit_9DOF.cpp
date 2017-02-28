#include "Adafruit_9DOF.h"

Adafruit_IMU::Adafruit_IMU()
{
  accel = Adafruit_LSM303_Accel_Unified(30301);
  mag   = Adafruit_LSM303_Mag_Unified(30302);
}

void Adafruit_IMU::init()
{
  accel.begin();
  mag.begin();
}

void Adafruit_IMU::update()
{
  accel.getEvent(&accel_event);
  dof.accelGetOrientation(&accel_event, &orientation);
  mag.getEvent(&mag_event);
  dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
}

