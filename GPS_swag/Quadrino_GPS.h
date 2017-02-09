#include <Wire.h>
#include "Arduino.h"
#include "gps_registers.h"

#define QGPS_I2C_ADDRESS        0x20                      //7 bit address 0x40 write, 0x41 read

class Quadrino_GPS
{
public:
  Quadrino_GPS();
  void init();
  void update();
  
  STATUS_REGISTER status;
  uint16_t alt;
  double lat, lon;

private:
  uint8_t version=0x1;
  GPS_COORDINATES location;
  GPS_DATETIME utc;
  unsigned long lastUpdate;

  bool gps_read_registers(uint8_t addr, void* variable, int length);
  uint8_t gps_read_u8(uint8_t addr);
  void gps_read_status();
  inline void gps_read_location();
  inline void gps_read_altitude();
  inline bool gps_read_utc_datetime(GPS_DATETIME& dt);
  inline uint32_t gps_read_utc_tow();
  inline uint16_t gps_read_utc_week();
};
