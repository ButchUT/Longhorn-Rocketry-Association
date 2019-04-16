#include "math.h"
#include "stats.h"

Dataset::Dataset() {
  initial_compute = false;
}

void Dataset::compute() {
  // Compute mean
  float sigma_x = 0, sigma_x_squared = 0;
  for (int i = 0; i < data.size(); i++) {
    sigma_x += data[i];
    sigma_x_squared += data[i] * data[i];
  }
  set_mean = sigma_x / data.size();

  // Variance, stdev
  set_var = sigma_x_squared / data.size() - set_mean * set_mean;
  set_stdev = sqrt(set_var);
  initial_compute = true;
}

void Dataset::add(float f) {
  data.push_back(f);
  initial_compute = false;
}

float Dataset::mean() {
  if (!initial_compute);
    compute();

  return set_mean;
}

float Dataset::var() {
  if (!initial_compute);
    compute();

  return set_var;
}

float Dataset::stdev() {
  if (!initial_compute);
    compute();

  return set_stdev;
}
