#ifndef HARDWARE_BAROMETER_H
#define HARDWARE_BAROMETER_H

#include <iostream>
#include <string>

struct BarometerData {
  float pressure;
  float altitude;
  float temperature;
};

class Barometer {
public:
  /**
    @brief Acknowledge that the barometer exists, wait for initialize
  */
  constexpr Barometer(void) {};

  /**
   * @brief Destroy the barometer
   */
  virtual ~Barometer(void);

  /**
   * @brief One-time initialization
   */
  virtual void initialize();

  /**
   * @brief Retrieve the most recent data
   */
  virtual struct BarometerData read() const = 0;

  /**
   * @brief Get telemetry about status
   */
  virtual std::string get_logging_data() const;
};

#endif
