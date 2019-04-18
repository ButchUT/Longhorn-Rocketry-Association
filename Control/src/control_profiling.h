#ifndef CONTROL_PROFILING_H
#define CONTROL_PROFILING_H

/**
  Relates an input to an output in some useful way.
*/
class Profile {
public:
  virtual float get(float x) = 0;
};

/**
  Relates an input to an output with a binomial of variable degree. Also
  observes a set of low and high bounds that serve to impose limits on the
  range of the function.
*/
class BinomialProfile: public Profile {
protected:
  float x1, x2, y1, y2;
  float exp, a, b;

public:
  /**
    Generates the profile and fits the binomial between the specified
    bounds. Bounds must be valid (i.e. max > min, etc.).

    @param x_min input value past which maximum output is returned
    @param x_max input value before which minimum output is returned
    @param y_min minimum output
    @param y_max maximum output
    @param power degree of the binomial
  */
  BinomialProfile(float x_min, float x_max, float y_min, float y_max,
    float power);

  /**
    Produces output according to this piecewise function:
        f(x) = { y_max if x <= x_min
               { y_min if x >= x_max
               { a*x^e+b otherwise

    @param x input
    @return y
  */
  float get(float x);
};

#endif
