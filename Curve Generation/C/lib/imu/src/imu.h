#ifndef _CURVE_GEN_IMU_H
#define _CURVE_GEN_IMU_H

#include <iostream>
#include <string>

struct ImuData {
  float gx, gy, gz; // Degrees/sec
  float ax, ay, az; // Gs
  float mx, my, mz; // Gs
};

class Imu {
public:
  constexpr Imu(void) {};

  virtual ~Imu(void);

  virtual void Initialize();

  virtual struct ImuData Read() const = 0;

  virtual std::string GetLoggingData() const;
};

#endif
