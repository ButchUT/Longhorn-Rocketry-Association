#include "mock_barometer_mpl.h"
#include <sstream>

void MockBarometerWrapper::initialize() {}

void MockBarometerWrapper::setTimestep(float t) {
  timestep = t;
}

void MockBarometerWrapper::setSimulator(FlightSimulator *sim) {
  flightsim = sim;
}

void MockBarometerWrapper::setBarometerNoise(NoiseGenerator *gen) {
  pressure_noise = gen;
}

void MockBarometerWrapper::setAltimeterNoise(NoiseGenerator *gen) {
  altitude_noise = gen;
}

void MockBarometerWrapper::setThermometerNoise(NoiseGenerator *gen) {
  temp_noise = gen;
}

void MockBarometerWrapper::setNoise(NoiseGenerator *gen) {
  setBarometerNoise(gen);
  setAltimeterNoise(gen);
  setThermometerNoise(gen);
}

struct BarometerData MockBarometerWrapper::read() const
{
  struct BarometerData data;

  // Note: flightsim currently doesn't generate all necessary barometer data
  data.altitude = flightsim->get_rocket_altitude();

  return data;
}
