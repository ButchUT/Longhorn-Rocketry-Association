#include "barometer_mpl.h"

#include <sstream>

#ifdef ARDUINO
  #include <Arduino.h>
#endif

#ifdef ARDUINO
  Adafruit_MPL3115A2 BarometerWrapper::barometer_ = Adafruit_MPL3115A2();
#endif

void BarometerWrapper::initialize() {
#ifdef ARDUINO
  BarometerWrapper::barometer_.begin();
#endif
}

struct BarometerData BarometerWrapper::read() const {
  struct BarometerData data;

#ifdef ARDUINO
  data.pressure = BarometerWrapper::barometer_.getPressure();
  data.altitude = BarometerWrapper::barometer_.getAltitude();
  data.temperature = BarometerWrapper::barometer_.getTemperature();
#endif

  return data;
}
