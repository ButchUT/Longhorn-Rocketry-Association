#include "barometer_mpl.h"

#include <sstream>
#ifdef ARDUINO
  #include <Arduino.h>
#endif

#ifdef ARDUINO
Adafruit_MPL3115A2 BarometerWrapper::barometer_ = Adafruit_MPL3115A2();
#endif // ARDUINO

void BarometerWrapper::Initialize() {
#ifdef ARDUINO
  BarometerWrapper::barometer_.begin();
#endif // ARDUINO
}

struct BarometerData BarometerWrapper::Read() const {
  struct BarometerData data;
#ifdef ARDUINO
  data.pressure = BarometerWrapper::barometer_.getPressure();
  data.altitude = BarometerWrapper::barometer_.getAltitude();
  data.temperature = BarometerWrapper::barometer_.getTemperature();

#else
  data.pressure = 0.0;
  data.altitude = 0.0;
  data.temperature = 0.0;

#endif // ARDUINO
  return data;
}
