#include "control_profiling.h"

#include <math.h>

BinomialProfile::BinomialProfile(float x1, float x2, float y1, float y2,
    float exp) {
  float x1r = pow(x1, exp), x2r = pow(x2, exp);
  a = (y2 - y1) / (1 / x2r - 1 / x1r);
  b = y1 - a / x1r;
  this->x1 = x1;
  this->x2 = x2;
  this->y1 = y1;
  this->y2 = y2;
  this->exp = exp;
}

float BinomialProfile::get(float x)
{
  if (x <= x1)
    return y1;
  else if (x >= x2)
    return y2;

  return a / pow(x, exp) + b;
}
