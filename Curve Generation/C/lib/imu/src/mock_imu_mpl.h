#ifndef _CURVE_GEN_MOCK_IMU_MPL_H
#define _CURVE_GEN_MOCK_IMU_MPL_H

#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <random>
#endif

#include "imu.h"
#include <string>

namespace imu {
  const float kMinimumGyro = -5000;
  const float kMaximumGyro = 5000;

  const float kMinimumAccel = -25;
  const float kMaximumAccel = 25;

  const float kMinimumMag = -0.1;
  const float kMaximumMag = 0.1;
}

class MockImuWrapper : public Imu {
public:
  constexpr MockImuWrapper(void) {}

  void Initialize();

  struct ImuData Read() const;

#ifndef ARDUINO
protected:
  static std::default_random_engine generator_;
  static std::uniform_real_distribution<float> gyro_distribution_;
  static std::uniform_real_distribution<float> accel_distribution_;
  static std::uniform_real_distribution<float> mag_distribution_;
#endif
};

#endif
