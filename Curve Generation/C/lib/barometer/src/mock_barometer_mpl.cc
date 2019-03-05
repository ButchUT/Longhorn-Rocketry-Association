#include "mock_barometer_mpl.h"

// Note - Some function (`_gettimeofday()` isn't defined properly when
// building for teensy)
#ifndef ARDUINO
#include <chrono>
#endif // ARDUINO

#include <sstream>

#ifndef ARDUINO
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

std::default_random_engine MockBarometerWrapper::generator_ = std::default_random_engine(seed);

std::uniform_real_distribution<float> MockBarometerWrapper::pressure_distribution_ = std::uniform_real_distribution<float>(
  baro::kMinimumPressure, baro::kMaximumPressure);
std::uniform_real_distribution<float> MockBarometerWrapper::altitude_distribution_ = std::uniform_real_distribution<float>(
  baro::kMinimumAltitude, baro::kMaximumAltitude);
std::uniform_real_distribution<float> MockBarometerWrapper::temperature_distribution_ = std::uniform_real_distribution<float>(
  baro::kMinimumTemperature, baro::kMaximumTemperature);
#endif // ARDUINO

void MockBarometerWrapper::Initialize()
{
#ifdef ARDUINO
  randomSeed(millis());
#endif // ARDUINO
}

struct BarometerData MockBarometerWrapper::Read() const
{
  struct BarometerData data;

#ifdef ARDUINO
  data.pressure = (random(0, 1000) / 1000) * (
    baro::kMaximumPressure - baro::kMinimumPressure) + baro::kMinimumPressure;
  data.altitude = (random(0, 1000) / 1000) * (
    baro::kMaximumAltitude - baro::kMinimumAltitude) + baro::kMinimumAltitude;
  data.temperature = (random(0, 1000) / 1000) * (
    baro::kMaximumTemperature - baro::kMinimumTemperature) + baro::kMinimumTemperature;
#else
  std::uniform_real_distribution<float> pressure = MockBarometerWrapper::pressure_distribution_;
  std::uniform_real_distribution<float> altitude = MockBarometerWrapper::altitude_distribution_;
  std::uniform_real_distribution<float> temperature = MockBarometerWrapper::temperature_distribution_;

  data.pressure = pressure(MockBarometerWrapper::generator_);
  data.altitude = altitude(MockBarometerWrapper::generator_);
  data.temperature = temperature(MockBarometerWrapper::generator_);
#endif // ARDUINO

  return data;
}
