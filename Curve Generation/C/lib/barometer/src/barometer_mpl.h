/**
 * @file barometer_mpl.h
 * @brief Setup, logging, and tear down for the barometer
 */

#ifndef _CURVE_GEN_BAROMETER_MPL_H
#define _CURVE_GEN_BAROMETER_MPL_H

#include <string>

#ifdef ARDUINO
  #include <Adafruit_MPL3115A2.h>
#endif

#include "barometer.h"

class BarometerWrapper : public Barometer {
public:
  /**
   * @brief Construct a BarometerWrapper instance
   * 
   */
  constexpr BarometerWrapper(void) {}

  /**
   * @brief Initialize the barometer.
   * 
   * This mainly just involves a call to `begin()` on the real barometer
   * instance
   */
  void Initialize();

  /**
   * @brief Read and return sensor data
   * 
   * @return struct BarometerData 
   */
  struct BarometerData Read() const;

#ifdef ARDUINO
protected:
  /**
   * @brief Static barometer instance
   * @static
   */
  static Adafruit_MPL3115A2 barometer_;
#endif // ARDUINO

};

#endif // _CURVE_GEN_BAROMETER_MPL_H
