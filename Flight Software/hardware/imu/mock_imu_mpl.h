#ifndef HARDWARE_MOCK_IMU_MPL_H
#define HARDWARE_MOCK_IMU_MPL_H

#include "flightsim.h"
#include "imu.h"
#include "noise.h"
#include <string>

class MockImuWrapper : public Imu {
public:
  MockImuWrapper() {}

  ~MockImuWrapper();

  void initialize();

  struct ImuData read() const;

  /**
    @brief Sets the time value used for noise generation
  */
  void setTimestep(float t);

  /**
    @brief Attaches a flight simulator from which to derive kinematical data
    for readings (mandatory)
  */
  void setSimulator(FlightSimulator *sim);

  /**
    @brief Attaches a noise source to future gyroscope readings
  */
  void setGyroscopeNoise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to future accelerometer readings
  */
  void setAccelerometerNoise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to future magnetometer readings
  */
  void setMagnetometerNoise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to all future readings
  */
  void setNoise(NoiseGenerator *gen);

protected:
  float timestep;
  FlightSimulator *flightsim;
  NoiseGenerator *gyro_noise, *accel_noise, *mag_noise;
};

#endif
