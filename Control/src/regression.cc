#include "regression.h"

PolynomialRegressor::PolynomialRegressor(size_t pointCount, int order) {
  n = order;
  np1 = n + 1;
  np2 = n + 2;
  tnp1 = 2 * n + 1;

  X = std::vector<float>(tnp1);
  Y = std::vector<float>(np1);
  a = std::vector<float>(np1);
  B = std::vector<std::vector<float>>(np1, std::vector<float>(np2, 0));
}

/**
  A version of Chris Engelsma's polynomial regression implementation with
  miscellaneous optimizations made.
  https://gist.github.com/chrisengelsma/108f7ab0a746323beaaf7d6634cf4add
*/
void PolynomialRegressor::fit(const std::vector<float> &x,
  const std::vector<float> &y, std::vector<float> &coeffs) {
  size_t N = x.size();

  // X = vector that stores values of sigma(xi^2n)
  for (int i = 0; i < tnp1; i++) {
    X[i] = 0;
    for (int j = 0; j < N; j++)
      X[i] += pow(x[j], i);
  }

  // B = normal augmented matrix that stores the equations
  for (int i = 0; i <= n; i++)
    for (int j = 0; j <= n; j++)
      B[i][j] = X[i + j];

  // Y = vector to store values of sigma(xi^n * yi)
  for (int i = 0; i < np1; i++) {
    Y[i] = 0.0;
    for (int j = 0; j < N; j++)
      Y[i] += pow(x[j], i) * y[j];
  }

  // Load values of Y into last column of B
  for (int i = 0; i <= n; i++)
    B[i][np1] = Y[i];

  // Pivoting of the B matrix
  for (int i = 0; i < np1; i++)
    for (int k = i + 1; k < np1; k++)
      if (B[i][i] < B[k][i])
        for (int j = 0; j <= np1; j++) {
          tmp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = tmp;
        }

  // Perform Gaussian elimination
  // (1) Make all elements below the pivot equals to zero
  //     or eliminate the variable.
  for (int i = 0; i < n; i++)
    for (int k = i + 1; k < np1; k++) {
      float t = B[k][i] / B[i][i];
      for (int j = 0; j <= np1; j++)
        B[k][j] -= t * B[i][j]; // (1)
    }

  // Back substitution
  // (1) Set the variable as the rhs of last equation
  // (2) Subtract all lhs values except the target coefficient
  // (3) Divide rhs by coefficient of variable being calculated
  for (int i = n; i >= 0; i--) {
    a[i] = B[i][np1]; // (1)
    for (int j = 0; j < np1; j++)
      if (j != i)
        a[i] -= B[i][j] * a[j]; // (2)
    a[i] /= B[i][i]; // (3)
  }

  coeffs.resize(a.size());
  for (size_t i = 0; i < a.size(); i++)
    coeffs[i] = a[i];
}

ExponentialRegressor::ExponentialRegressor(size_t point_count) {
  lny = new float[point_count];
}

ExponentialRegressor::~ExponentialRegressor() {
  delete lny;
}

void ExponentialRegressor::fit(const std::vector<float> &x,
  const std::vector<float> &y, std::vector<float> &coeffs) {

  size_t N = x.size();

  // Calculate values of ln(y_i)
  for (int i = 0; i < N; i++)
    lny[i] = log(y[i]);

  float x_sum = 0, x2_sum = 0, y_sum = 0, xy_sum = 0;
  for (int i = 0; i < N; i++) {
    x_sum += x[i];
    x2_sum += x[i] * x[i];
    y_sum += lny[i];
    xy_sum += x[i] * lny[i];
  }

  float a = (N * xy_sum - x_sum * y_sum) / (N * x2_sum - x_sum * x_sum);
  float b = (x2_sum * y_sum - x_sum * xy_sum) / (x2_sum * N - x_sum * x_sum);
  float c = pow(E, b);

  coeffs[0] = a;
  coeffs[1] = c;
}
