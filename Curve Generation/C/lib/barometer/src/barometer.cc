#include "barometer.h"

#include <sstream>

Barometer::~Barometer() {}

void Barometer::Initialize() {}

std::string Barometer::GetLoggingData() const {
  struct BarometerData data = this->Read();
  std::stringstream output;
  output << data.pressure << ", " << data.altitude << ", " << data.temperature << ", ";

  return output.str();
}
