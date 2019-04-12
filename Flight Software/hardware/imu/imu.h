#ifndef HARDWARE_IMU_H
#define HARDWARE_IMU_H

#include <iostream>
#include <string>

struct ImuData {
  float gx, gy, gz; // Gyroscope
  float ax, ay, az; // Accelerometer
  float mx, my, mz; // Magnetometer
};

class Imu {
public:
  /**
    @brief Acknowledge that the IMU exists, wait to initialize
  */
  constexpr Imu(void) {};

  /**
    @brief Destroy the IMU
  */
  virtual ~Imu(void);

  /**
    @brief One-time initialization
  */
  virtual void initialize();

  /**
    @brief Fetch the most recent data
  */
  virtual struct ImuData read() const = 0;

  /**
    @brief Get telemetry about status
  */
  virtual std::string get_logging_data() const;
};

#endif
