/**
 * @file mock_barometer_mpl.h
 * @brief Mock barometer object
 */

#ifndef _CURVE_GEN_MOCK_BAROMETER_MPL_H
#define _CURVE_GEN_MOCK_BAROMETER_MPL_H

namespace baro
{
  const float kMinimumPressure = 100.00f;
  const float kMaximumPressure = 500.00f;

  const float kMinimumAltitude = 1200.0f;
  const float kMaximumAltitude = 3000.0f;

  const float kMinimumTemperature = 273.706f;
  const float kMaximumTemperature = 325.039f;
}

#ifdef ARDUINO
  #include <Arduino.h>

  // Note - The architecture couldn't fathom some of the functions in
  // <random>, so I used Arduino's `random()` function
#else
  #include <random>
#endif
#include <string>

#include "barometer.h"

class MockBarometerWrapper : public Barometer {
public:
  /**
   * @brief Construct a BarometerWrapper instance
   * 
   */
  constexpr MockBarometerWrapper(void) {}

  /**
   * @brief Initialize the barometer.
   * 
   * Within the mock, this creates the random data generators
   */
  void Initialize();

  /**
   * @brief Read and return sensor data
   * 
   * @return struct BarometerData 
   */
  struct BarometerData Read() const;

#ifndef ARDUINO
protected:
  static std::default_random_engine generator_;
  static std::uniform_real_distribution<float> pressure_distribution_;
  static std::uniform_real_distribution<float> altitude_distribution_;
  static std::uniform_real_distribution<float> temperature_distribution_;
#endif
};

#endif // _CURVE_GEN_MOCK_BAROMETER_MPL_H
