#ifndef PROFILING_H
#define PROFILING_H

/**
    Relates an input to brake step size in some useful way.
*/
class BrakeProfile
{
public:
    virtual float get_step_size(float x) = 0;
};

/**
    Relates an input to step size with a binomial of variable degree (a function
    of the form (y=ax^e+b).

    More often than not, the "input" will be rocket velocity.
*/
class BinomialBrakeProfile: public BrakeProfile
{
protected:
    float minBrakeStep, maxBrakeStep;
    float minInput, maxInput;
    float e, a, b;

public:
    /**
        Generates the profile and fits the binomial between the specified
        bounds. Bounds must be valid (i.e. max > min, etc.).

        @param minBrakeStep minimum brake step size
        @param maxBrakeStep maximum brake step size
        @param minInput input below which maxBrakeStep is used
        @param maxInput input above which minBrakeStep is used
        @param degree of the binomial
    */
    BinomialBrakeProfile(float minBrakeStep, float maxBrakeStep, float minInput,
        float maxInput, float power);

    /**
        Gets step size as a function of some input variable. The function looks
        something like this:
            s(x) = { maxBrakeStep if x <= minInput
                   { minBrakeStep if x >= maxInput
                   { a*x^e+b otherwise

        @param x input
        @return brake step size
    */
    float get_step_size(float x);
};

#endif
