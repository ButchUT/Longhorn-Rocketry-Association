/**
 * @file test_barometer.cc
 *
 * @brief Tests the *mock* barometer library 
 */

#include <iostream>

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

#include <memory>
#include <unity.h>

#include "barometer.h"
#include "mock_barometer_mpl.h"

void test_read(void) {
  std::shared_ptr<Barometer> barometer = std::shared_ptr<Barometer>(new MockBarometerWrapper());

  barometer->Initialize();

  struct BarometerData data = barometer->Read();

  std::cout << "pressure: " << data.pressure << ", altitude: " << data.altitude << ", temperature: " << data.temperature << '\n';
  TEST_ASSERT(baro::kMinimumPressure <= data.pressure &&
    data.pressure <= baro::kMaximumPressure);
  TEST_ASSERT(baro::kMinimumAltitude <= data.altitude &&
    data.altitude <= baro::kMaximumAltitude);
  TEST_ASSERT(baro::kMinimumTemperature <= data.temperature &&
    data.temperature <= baro::kMaximumTemperature);
}

/**
 * Portable function to run all the tests
 */
void run_tests() {
  UNITY_BEGIN();

  RUN_TEST(test_read);

  UNITY_END();
}

#ifdef ARDUINO

/**
 * Setup for arduino framework. Called once at the beginning of testing
 */
void setup() {
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);

  run_tests();
}

/**
 * Loop function for arduino framework. Called repeatedly
 */
void loop() {}

#else

/**
 * Entypoint for the test code
 * @param argc integer representing the number of command line arguments.
 *   Not relevant in a testing context
 * @param argv string array of the command line arguments. Also not relevant
 *  in a testing environment
 * @returns exit code of the program
 */
int main(int argc, char* argv[]) {
  run_tests();
  return 0;
}

#endif
