#include <iostream>
#include <memory>
#include <unity.h>

#include "imu.h"
#include "mock_imu_mpl.h"

#ifdef ARDUINO

#include <Arduino.h>

namespace std {
  void __throw_length_error( char const*e ) {
    Serial.print("Length Error :");
    Serial.println(e);
    while(1);
  }
}

#endif

void test_read(void) {
  std::shared_ptr<Imu> imu =
    std::shared_ptr<Imu>(new MockImuWrapper());

  imu->Initialize();

  struct ImuData data = imu->Read();

  std::cout << imu->GetLoggingData();

  TEST_ASSERT(imu::kMinimumGyro <= data.gx && data.gx <= imu::kMaximumGyro);
  TEST_ASSERT(imu::kMinimumGyro <= data.gy && data.gy <= imu::kMaximumGyro);
  TEST_ASSERT(imu::kMinimumGyro <= data.gz && data.gz <= imu::kMaximumGyro);
  TEST_ASSERT(imu::kMinimumAccel <= data.ax && data.ax <= imu::kMaximumAccel);
  TEST_ASSERT(imu::kMinimumAccel <= data.ay && data.ay <= imu::kMaximumAccel);
  TEST_ASSERT(imu::kMinimumAccel <= data.az && data.az <= imu::kMaximumAccel);
  TEST_ASSERT(imu::kMinimumMag <= data.mx && data.mx <= imu::kMaximumMag);
  TEST_ASSERT(imu::kMinimumMag <= data.my && data.my <= imu::kMaximumMag);
  TEST_ASSERT(imu::kMinimumMag <= data.mz && data.mz <= imu::kMaximumMag);
}

void run_tests() {
  UNITY_BEGIN();
  RUN_TEST(test_read);
  UNITY_END();
}

#ifdef ARDUINO

void setup() {
  delay(2000);
  run_tests();
}

void loop() {}

#else

int main(int argc, char* argv[]) {
  run_tests();
  return 0;
}

#endif
