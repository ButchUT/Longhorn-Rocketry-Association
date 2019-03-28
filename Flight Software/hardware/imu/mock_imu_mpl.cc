#include "mock_imu_mpl.h"
#include <sstream>
#include <vector>

void MockImuWrapper::initialize() {
  flightsim = nullptr;
  gyro_noise = accel_noise = mag_noise = nullptr;
}

MockImuWrapper::~MockImuWrapper() {
  delete gyro_noise;
  delete accel_noise;
  delete mag_noise;
}

void MockImuWrapper::setTimestep(float t) {
  timestep = t;
}

void MockImuWrapper::setSimulator(FlightSimulator *sim) {
  flightsim = sim;
}

void MockImuWrapper::setGyroscopeNoise(NoiseGenerator *gen) {
  gyro_noise = gen;
}

void MockImuWrapper::setAccelerometerNoise(NoiseGenerator *gen) {
  accel_noise = gen;
}

void MockImuWrapper::setMagnetometerNoise(NoiseGenerator *gen) {
  mag_noise = gen;
}

void MockImuWrapper::setNoise(NoiseGenerator *gen) {
  setGyroscopeNoise(gen);
  setAccelerometerNoise(gen);
  setMagnetometerNoise(gen);
}

struct ImuData MockImuWrapper::read() const {
  std::vector<float> gyro(flightsim::AXES_COUNT);
  std::vector<float> accel(flightsim::AXES_COUNT);

  // Read from flightsim
  if (flightsim != nullptr) {
    gyro = flightsim->get_rocket_gyro();
    accel = flightsim->get_rocket_acceleration();
  }

  // Apply noise
  if (gyro_noise != nullptr)
    for (int i = 0; i < gyro.size(); i++)
      gyro[i] += gyro_noise->gen(timestep);

  if (accel_noise != nullptr)
    for (int i = 0; i < accel.size(); i++)
      accel[i] += accel_noise->gen(timestep);

  struct ImuData data;
  data.gx = gyro[flightsim::X_AXIS];
  data.gy = gyro[flightsim::Y_AXIS];
  data.gz = gyro[flightsim::Z_AXIS];
  data.ax = accel[flightsim::X_AXIS];
  data.ay = accel[flightsim::Y_AXIS];
  data.az = accel[flightsim::Z_AXIS];

  return data;
}
