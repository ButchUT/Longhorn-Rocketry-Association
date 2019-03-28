#include "abc_preconfig.h"
#include "abc_profiling.h"

AirbrakeController* make_2019_sac_abc() {
  const float TARGET_ALTITUDE = 5600;
  const float MIN_VELOCITY = 150;
  const float MAX_VELOCITY = 343 * 1;
  const float MIN_BRAKE_STEP = 0.075;
  const float MAX_BRAKE_STEP = 0.1;
  const float EXP = -1;
  const size_t BOUNDS_HISTORY_SIZE = 500;

  BinomialBrakeProfile *profile = new BinomialBrakeProfile(MIN_BRAKE_STEP,
    MAX_BRAKE_STEP, MIN_VELOCITY, MAX_VELOCITY, EXP);
  AirbrakeController *abc = new AirbrakeController(TARGET_ALTITUDE, profile,
    BOUNDS_HISTORY_SIZE);

  return abc;
}
