
#include "density_calculator.h"

#include <cassert>

#include "interpolate.h"

/**
 * Note(Sam): This array is static
 * https://en.cppreference.com/w/cpp/language/storage_duration#Storage_duration
 */
std::pair<double, double> kPressureTable[kNumPressureTableValues] = {
  { 0, 101.3 }, { 152, 99.4 }, { 305, 97.6 },
  { 457, 95.8 }, { 610, 94.1 }, { 762, 92.4 },
  { 914 , 90.7 }, { 1067, 89.1 }, { 1219, 87.4 },
  { 1372, 85.8 }, { 1524, 84.3 }, { 1676, 82.7 },
  { 1829, 81.1 }, { 1981, 79.6 }, { 2134, 78.1 },
  { 2286, 76.7 }, { 2438, 75.2 }, { 2591, 73.8 },
  { 2743, 72.3 }, { 2896, 71.0 }, { 3048, 69.6 },
  { 3200, 68.3 }, { 3353, 67.0 }, { 3505, 65.7 },
  { 3658, 64.4 }, { 3810, 63.1 }, { 3962, 61.9 },
  { 4115, 60.7 }, { 4267, 59.5 }, { 4420, 58.3 },
  { 4572, 57.1 }, { 4724, 56.0 }, { 4877, 54.8 },
  { 5029, 53.8 }, { 5182, 52.7 }, { 5334, 51.5 },
  { 5486, 50.6 }, { 5639, 49.5 }, { 5791, 48.6 },
  { 5944, 47.5 }, { 6096, 46.6 }, { 6248, 45.6 },
  { 6401, 44.6 }, { 6553, 43.7 }, { 6706, 42.8 },
  { 6858, 41.9 }, { 7010, 41.0 }, { 7163, 40.1 },
  { 7315, 39.3 }, { 7468, 38.4 }, { 7620, 37.6 },
  { 7772, 36.8 }, { 7925, 36.0 }, { 8077, 35.2 },
  { 8230, 34.5 }, { 8382, 33.7 }, { 8534, 32.9 },
  { 8687, 32.2 }, { 8839, 31.5 }, { 8992, 30.8 },
  { 9144, 30.1 }
};

/**
 * Ideal (universal) gas constant. Expressed in Joules / (mol * Kelvin).
 */
const double kR = 8.3144568;

/**
 * Molar mass of air. Expressed in Kilograms / mol.
 */
const double kM = 0.0289644;

/**
 * Earth surface gravitational acceleration.
 * Expressed in meters / (seconds ^ 2).
 */
const double kG = 9.80665;

/**
 * Temperature lapse rate. Expressed in Kelvin / meter..
 */
const double kL = 0.0065;

/**
 * Sea level standard temperature. Expressed in Kelvin.
 */
const double kT_0 = 288.15;

/**
 * Sea level standard atmospheric pressure. Expressed in kilopascals.
 */
const double kP_0 = 101.325;

double calculate_density(double start_height, double height) {
  /*assert(start_height >= 0);
  assert(height >= 0);*/

  double altitude = start_height + height;
  double temperature = kT_0 - kL * altitude;

  double pressure = Interp(kPressureTable, kNumPressureTableValues, altitude);

  // Converting from kilopascals to Pascals,
  // which is expressed in (Joules / meter ^ 3)
  pressure *= 1000;

  return (pressure * kM) / (kR * temperature);
}
