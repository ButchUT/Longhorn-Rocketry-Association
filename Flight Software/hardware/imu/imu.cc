#include "imu.h"

#include <sstream>

Imu::~Imu() {}

void Imu::initialize() {}

std::string Imu::get_logging_data() const {
  struct ImuData data = this->read();
  std::stringstream ostream;

  ostream << "g=<" << data.gx << ", " << data.gy << ", " << data.gz << ">" << std::endl <<
      "a=<" << data.ax << ", " << data.ay << ", " << data.az << ">" << std::endl <<
      "m=<" << data.mx << ", " << data.my << ", " << data.mz << ">" << std::endl;

  return ostream.str();
}
