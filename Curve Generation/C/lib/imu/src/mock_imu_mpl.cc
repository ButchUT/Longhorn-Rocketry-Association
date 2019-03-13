#ifndef ARDUINO
  #include <chrono>
#endif

#include "mock_imu_mpl.h"
#include <sstream>
#include <vector>

#ifndef ARDUINO
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine MockImuWrapper::generator_ =
    std::default_random_engine(seed);
  std::uniform_real_distribution<float> MockImuWrapper::gyro_distribution_ =
    std::uniform_real_distribution<float>(imu::kMinimumGyro,
      imu::kMaximumGyro);
  std::uniform_real_distribution<float> MockImuWrapper::accel_distribution_ =
    std::uniform_real_distribution<float>(imu::kMinimumAccel,
      imu::kMaximumAccel);
  std::uniform_real_distribution<float> MockImuWrapper::mag_distribution_ =
    std::uniform_real_distribution<float>(imu::kMinimumMag,
      imu::kMaximumMag);
#endif

void MockImuWrapper::Initialize() {
#ifdef ARDUINO
  randomSeed(millis());
#endif
}

struct ImuData MockImuWrapper::Read() const {
  struct ImuData data;

  if (flightsim != nullptr) {
    std::vector<float> gyro = flightsim->get_gyro();
    std::vector<float> accel = flightsim->get_acceleration();
    data.gx = gyro[flightsim::X_AXIS];
    data.gy = gyro[flightsim::Y_AXIS];
    data.gz = gyro[flightsim::Z_AXIS];
    data.ax = accel[flightsim::X_AXIS];
    data.ay = accel[flightsim::Y_AXIS];
    data.az = accel[flightsim::Z_AXIS];
  } else {
  #ifdef ARDUINO
    data.gx = (random(0, 1000) / 1000.0) *
      (imu::kMaximumGyro - imu::kMinimumGyro) + imu::kMinimumGyro;
    data.gy = (random(0, 1000) / 1000.0) *
      (imu::kMaximumGyro - imu::kMinimumGyro) + imu::kMinimumGyro;
    data.gz = (random(0, 1000) / 1000.0) *
      (imu::kMaximumGyro - imu::kMinimumGyro) + imu::kMinimumGyro;
    data.ax = (random(0, 1000) / 1000.0) *
      (imu::kMaximumAccel - imu::kMinimumAccel) + imu::kMinimumAccel;
    data.ay = (random(0, 1000) / 1000.0) *
      (imu::kMaximumAccel - imu::kMinimumAccel) + imu::kMinimumAccel;
    data.az = (random(0, 1000) / 1000.0) *
      (imu::kMaximumAccel - imu::kMinimumAccel) + imu::kMinimumAccel;
    data.mx = (random(0, 1000) / 1000.0) *
      (imu::kMaximumMag - imu::kMinimumMag) + imu::kMinimumMag;
    data.my = (random(0, 1000) / 1000.0) *
      (imu::kMaximumMag - imu::kMinimumMag) + imu::kMinimumMag;
    data.mz = (random(0, 1000) / 1000.0) *
      (imu::kMaximumMag - imu::kMinimumMag) + imu::kMinimumMag;
  #else
    std::uniform_real_distribution<float> gyro =
      MockImuWrapper::gyro_distribution_;
    std::uniform_real_distribution<float> accel =
      MockImuWrapper::accel_distribution_;
    std::uniform_real_distribution<float> mag =
      MockImuWrapper::mag_distribution_;

    data.gx = gyro(MockImuWrapper::generator_);
    data.gy = gyro(MockImuWrapper::generator_);
    data.gz = gyro(MockImuWrapper::generator_);
    data.ax = accel(MockImuWrapper::generator_);
    data.ay = accel(MockImuWrapper::generator_);
    data.az = accel(MockImuWrapper::generator_);
    data.mx = mag(MockImuWrapper::generator_);
    data.my = mag(MockImuWrapper::generator_);
    data.mz = mag(MockImuWrapper::generator_);
  #endif
  }

  return data;
}

void MockImuWrapper::AttachSimulator(FlightSimulator *sim) {
  flightsim = sim;
}
