#include "preconfig.h"
#include "profiling.h"

AirBrakeController* make_2019_sac_abc()
{
    const float TARGET_ALTITUDE = 3000;
    const float MIN_VELOCITY = 50;
    const float MAX_VELOCITY = 343 * 1;
    const float MIN_BRAKE_STEP = 0.025;
    const float MAX_BRAKE_STEP = 0.1;
    const float EXP = -1;

    BinomialBrakeProfile *profile = new BinomialBrakeProfile(MIN_BRAKE_STEP,
        MAX_BRAKE_STEP, MIN_VELOCITY, MAX_VELOCITY, EXP);
    AirBrakeController *abc = new AirBrakeController(TARGET_ALTITUDE, profile);

    return abc;
}
