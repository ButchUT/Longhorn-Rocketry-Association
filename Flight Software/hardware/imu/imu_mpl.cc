#ifdef ARDUINO
  #include <Arduino.h>
#endif

#include "imu_mpl.h"
#include <sstream>

void ImuWrapper::initialize() {
#ifdef ARDUINO
  ImuWrapper::imu.settings.device.commInterface = IMU_MODE_I2C;
  ImuWrapper::imu.settings.device.mAddress = ImuWrapper::LSM9DS1_M;
  ImuWrapper::imu.settings.device.agAddress = ImuWrapper::LSM9DS1_AG;
  ImuWrapper::imu_.begin();
#endif
}

struct ImuData ImuWrapper::read() const {
  struct ImuData data;

#ifdef ARDUINO
  ImuWrapper::imu_.readGyro();
  ImuWrapper::imu_.readAccel();
  ImuWrapper::imu_.readMag();
  data.gx = ImuWrapper::imu_.calcGyro(ImuWrapper::imu_.gx);
  data.gy = ImuWrapper::imu_.calcGyro(ImuWrapper::imu_.gy);
  data.gz = ImuWrapper::imu_.calcGyro(ImuWrapper::imu_.gz);
  data.ax = ImuWrapper::imu_.calcAccel(ImuWrapper::imu_.ax);
  data.ay = ImuWrapper::imu_.calcAccel(ImuWrapper::imu_.ay);
  data.az = ImuWrapper::imu_.calcAccel(ImuWrapper::imu_.az);
  data.mx = ImuWrapper::imu_.calcMag(ImuWrapper::imu_.mx);
  data.my = ImuWrapper::imu_.calcMag(ImuWrapper::imu_.my);
  data.mz = ImuWrapper::imu_.calcMag(ImuWrapper::imu_.mz);
#else
  data.gx = data.gy = data.gz = 0;
  data.ax = data.ay = data.az = 0;
  data.mx = data.my = data.mz = 0;
#endif

  return data;
}
