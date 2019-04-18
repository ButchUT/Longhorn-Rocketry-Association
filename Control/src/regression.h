#ifndef CONTROL_REGRESSION_H
#define CONTROL_REGRESSION_H

#include <math.h>
#include <stdlib.h>
#include <vector>

class Regressor {
public:
  virtual void fit(const std::vector<float> &x, const std::vector<float> &y,
    std::vector<float> &coeffs) = 0;

  virtual void intersect(const std::vector<float> &a_coeffs,
    const std::vector<float> &b_coeffs, std::vector<float> &sols) = 0;
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

  void intersect(const std::vector<float> &a_coeffs,
    const std::vector<float> &b_coeffs, std::vector<float> &sols);
};

class ExponentialRegressor : public Regressor {
public:
  ExponentialRegressor(size_t point_count);

  void fit(const std::vector<float> &x, const std::vector<float> &y,
    std::vector<float> &coeffs);

  void intersect(const std::vector<float> &a_coeffs,
    const std::vector<float> &b_coeffs, std::vector<float> &sols);
};

#endif
