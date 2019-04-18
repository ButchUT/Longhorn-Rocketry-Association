#ifndef HARDWARE_MOCK_IMU_MPL_H
#define HARDWARE_MOCK_IMU_MPL_H

#include "flight_sim.h"
#include "imu.h"
#include "noise.h"
#include <string>
#include <vector>

class MockImuWrapper : public Imu {
public:
  MockImuWrapper() {}

  void initialize();

  /**
    @brief Get the most recent sensor data
  */
  struct ImuData read() const;

  /**
    @brief Sets the time value used for noise generation
  */
  void set_timestep(float t);

  /**
    @brief Attaches a flight simulator from which to derive kinematical data
    for readings (mandatory)
  */
  void set_simulator(FlightSimulator *sim);

  /**
    @brief Attaches a noise source to gyroscope readings on a particular axis
  */
  void set_gyro_noise(NoiseGenerator *gen, int axis);

  /**
    @brief Attaches a noise source to future gyroscope readings
  */
  void set_gyro_noise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to accelerometer readings on a particular
    axis
  */
  void set_acceleration_noise(NoiseGenerator *gen, int axis);

  /**
    @brief Attaches a noise source to future accelerometer readings
  */
  void set_acceleration_noise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to magnetometer readings on a particular axis
  */
  void set_magnetic_noise(NoiseGenerator *gen, int axis);

  /**
    @brief Attaches a noise source to future magnetometer readings
  */
  void set_magnetic_noise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to all future readings
  */
  void set_noise(NoiseGenerator *gen);

protected:
  float timestep;
  FlightSimulator *flightsim;
  std::vector<NoiseGenerator*> gyro_noise, accel_noise, mag_noise;
};

#endif
