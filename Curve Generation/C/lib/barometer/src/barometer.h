/**
 * @file barometer.h
 *
 * @brief Virtual top level barometer object
 */

#ifndef _CURVE_GEN_BAROMETER_H_
#define _CURVE_GEN_BAROMETER_H_

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
   * @brief Construct a Barometer instance.
   * 
   */
  constexpr Barometer(void) {};

  /**
   * @brief Destruct Barometer instance
   * 
   */
  virtual ~Barometer(void);

  /**
   * @brief Initialize the barometer.
   * 
   * This mainly just involves a call to `begin()` on the real barometer
   * instance
   */
  virtual void Initialize();

  /**
   * @brief Read and return sensor data
   * 
   * @return struct BarometerData 
   */
  virtual struct BarometerData Read() const = 0;

  /**
   * @brief Retrieve data in a string
   * 
   * @returns Output
   */
  virtual std::string GetLoggingData() const;
};

#endif // _CURVE_GEN_BAROMETER_H_
