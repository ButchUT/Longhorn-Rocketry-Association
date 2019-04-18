#include "control_util.h"

float util::fsign(float f) {
  return f < 0 ? -1 : (f > 0 ? 1 : 0);
}

float util::fconstrain(float f, float lower, float upper) {
  return f < lower ? lower : (f > upper ? upper : f);
}
