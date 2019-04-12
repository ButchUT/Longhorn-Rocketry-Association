#include "barometer.h"
#include <sstream>

Barometer::~Barometer() {}

void Barometer::initialize() {}

std::string Barometer::get_logging_data() const {
  struct BarometerData data = this->read();
  std::stringstream output;

  output << "<" << data.pressure << ", " << data.altitude << ", " <<
    data.temperature << ">";

  return output.str();
}
