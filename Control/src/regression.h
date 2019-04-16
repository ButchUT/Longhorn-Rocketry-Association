#ifndef CONTROL_REGRESSION_H
#define CONTROL_REGRESSION_H

#include <math.h>
#include <stdlib.h>
#include <vector>

class Regressor {
public:
  virtual void fit(const std::vector<float> &x, const std::vector<float> &y,
    std::vector<float> &coeffs) = 0;
};

class PolynomialRegressor : public Regressor {
protected:
  std::vector<float> X, Y, a;
  std::vector<std::vector<float>> B;

  int order, n, np1, np2, tnp1;
  float tmp;

public:
  /**
    One-time allocation of data structures relevant to regression.

    @param pointCount size of input point set
    @param order degree of polynomial
  */
  PolynomialRegressor(size_t pointCount, int order);

  /**
    Fits a non-linear relationship to a set of points using the
    least-squares approach.

    @param x x ordinates
    @param y y ordinates
    @param order polynomial degree
    @param coeffs receptacle for function coefficients
  */
  void fit(const std::vector<float> &x, const std::vector<float> &y,
    std::vector<float> &coeffs);
};

class ExponentialRegressor : public Regressor {
protected:
  const float E = exp(1.0);
  float *lny;

public:
  ExponentialRegressor(size_t point_count);

  ~ExponentialRegressor();

  void fit(const std::vector<float> &x, const std::vector<float> &y,
    std::vector<float> &coeffs);
};

#endif
