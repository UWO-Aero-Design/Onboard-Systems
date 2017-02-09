#include "Quadrino_GPS.h"

Quadrino_GPS::Quadrino_GPS()
{
  lastUpdate = millis();
}

void Quadrino_GPS::init()
{
  utc.date = utc.time = 0;

  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(I2C_GPS_COMMAND);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  
  // read the version of the GPS, it should be 21
  do {
    delay(100);
    version = gps_read_u8(I2C_GPS_REG_VERSION);
  } while(version==0 || version==0xff);
}

void Quadrino_GPS::update()
{
  // Update at a max of 20 Hz
  if (millis() - lastUpdate > 50) {
    gps_read_status();
    gps_read_altitude();
    gps_read_location();
    lastUpdate = millis();
  }
}

uint8_t Quadrino_GPS::gps_read_u8(uint8_t addr)
{
  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(QGPS_I2C_ADDRESS,1);
  return Wire.read();
}

void Quadrino_GPS::gps_read_status()
{
  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write((uint8_t)I2C_GPS_STATUS_00);
  Wire.endTransmission();
  Wire.requestFrom(QGPS_I2C_ADDRESS,1);
  byte b = Wire.read();
  status = *(STATUS_REGISTER*)&b;
}

bool Quadrino_GPS::gps_read_registers(uint8_t addr, void* variable, int length)
{
  byte* loc = (byte*)variable;
  Wire.beginTransmission(QGPS_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(QGPS_I2C_ADDRESS,length);
  if(length <= Wire.available()) {
    for(int i=0; i<length; i++)
      *loc++ = Wire.read();
    return true;
  } else
    return false;
}

inline void Quadrino_GPS::gps_read_location()
{
  gps_read_registers(I2C_GPS_LOCATION, &location, sizeof(GPS_COORDINATES));
  lat = (double)location.lat/10000000.0;
  lon = (double)location.lon/10000000.0;
}

inline void Quadrino_GPS::gps_read_altitude()
{
  gps_read_registers(I2C_GPS_ALTITUDE, &alt, sizeof(alt));
}

inline bool Quadrino_GPS::gps_read_utc_datetime(GPS_DATETIME& dt)
{
  return gps_read_registers(I2C_GPS_WEEK, &dt, sizeof(GPS_DATETIME  ));
}

inline uint32_t Quadrino_GPS::gps_read_utc_tow()
{
  uint32_t utc;
  return (gps_read_registers(I2C_GPS_TIME, &utc, sizeof(utc))) ? utc : 0;
}

inline uint16_t Quadrino_GPS::gps_read_utc_week()
{
  uint16_t utc;
  return (gps_read_registers(I2C_GPS_WEEK, &utc, sizeof(utc))) ? utc : 0;
}
