#ifndef _CURVE_GEN_IMU_MPL_H
#define _CURVE_GEN_IMU_MPL_H

#ifdef ARDUINO
  #include <SparkFunLSM9DS1.h>
#endif

#include "imu.h"
#include <string>

class ImuWrapper : public Imu {
public:
  constexpr ImuWrapper(void) {}

  void Initialize();

  struct ImuData Read() const;

#ifdef ARDUINO
protected:
  static LSM9DS1 imu_;
  static const int LSM9DS1_M = 0x1E; // Would be 0x1C if SDO_M is LOW
  static const int LSM9DS1_AG = 0x6B; // Would be 0x6A if SDO_AG is LOW
  static const float DECLINATION 3.44; // http://www.ngdc.noaa.gov/geomag-web/#declination
#endif
};

#endif
