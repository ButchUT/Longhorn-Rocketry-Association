#ifndef HARDWARE_BAROMETER_MPL_H
#define HARDWARE_BAROMETER_MPL_H

#include "barometer.h"

#include <string>

#ifdef ARDUINO
  #include <Adafruit_MPL3115A2.h>
#endif

class BarometerWrapper : public Barometer {
public:
  constexpr BarometerWrapper(void) {}

  void initialize();

  struct BarometerData read() const;

protected:
#ifdef ARDUINO
  static Adafruit_MPL3115A2 barometer_;
#endif
};

#endif
