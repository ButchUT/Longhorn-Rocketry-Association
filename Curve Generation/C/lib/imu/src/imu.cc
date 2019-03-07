#include "imu.h"
#include <sstream>

Imu::~Imu() {}

void Imu::Initialize() {}

std::string Imu::GetLoggingData() const {
  struct ImuData data = this->Read();
  std::stringstream ostream;
  ostream << "g=<" << data.gx << ", " << data.gy << ", " << data.gz << ">"
    << std::endl << "a=<" << data.ax << ", " << data.ay << ", " << data.az << ">"
    << std::endl << "m=<" << data.mx << ", " << data.my << ", " << data.mz << ">"
    << std::endl;
  return ostream.str();
}
