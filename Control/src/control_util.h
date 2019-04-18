#ifndef CONTROL_UTIL_H
#define CONTROL_UTIL_H

#include <math.h>

const float NIL = 0x7FC00000; // Standard NaN value used to indicate "no solution"
const float EULERS_NUMBER = exp(1.0);

namespace util {
  float fsign(float f);

  /**
    Clamps a float between some bounds.

    @param f value to constrain
    @param lower lower bound
    @param upper upper bound
  */
  float fconstrain(float f, float lower, float upper);
}

#endif
