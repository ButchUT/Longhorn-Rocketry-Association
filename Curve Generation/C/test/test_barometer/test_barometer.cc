/**
 * @file test_barometer.cc
 *
 * @brief Tests the *real* barometer library 
 */

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
#include "barometer_mpl.h"

void test_read(void) {
  std::shared_ptr<Barometer> barometer = std::shared_ptr<Barometer>(new BarometerWrapper());

  barometer->Initialize();
  struct BarometerData data = barometer->Read();

  // Note - If this code is running on the arduino, then this *will* test the
  // actual sensors. Use this section to test some default ranges or maybe set some
  // control settings to get back expected results.
  //
  // Adafruit barometer open source library
  // https://github.com/adafruit/Adafruit_MPL3115A2_Library/blob/master/Adafruit_MPL3115A2.h
  // Unity testing API
  // https://docs.platformio.org/en/latest/plus/unit-testing.html#api
#ifdef ARDUINO


  // Note - Testing the real barometer on a computer does nothing, no sensor
  // data is read
#else
  TEST_ASSERT_EQUAL_FLOAT(0.0, data.pressure);
  TEST_ASSERT_EQUAL_FLOAT(0.0, data.altitude);
  TEST_ASSERT_EQUAL_FLOAT(0.0, data.temperature);
#endif // ARDUINO

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
