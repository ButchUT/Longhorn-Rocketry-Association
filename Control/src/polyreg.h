#ifndef POLYREG_H
#define POLYREG_H

#include <stdlib.h>
#include <vector>

using namespace std;

class PolynomialRegression
{
private:
    vector<float> X, Y, a;
    vector<vector<float>> B;

    size_t N;
    int order, n, np1, np2, tnp1;
    float tmp;

public:
    /**
        One-time allocation of data structures relevant to regression.

        @param pointCount size of input point set
        @param order degree of polynomial
    */
    PolynomialRegression(size_t pointCount, int order);

    /**
        Fits a non-linear relationship to a set of points using the
        least-squares approach.

        @param x x ordinates
        @param y y ordinates
        @param order polynomial degree
        @param coeffs receptacle for function coefficients
    */
    void polyreg(const vector<float> &x, const vector<float> &y,
        vector<float> &coeffs);
};

#endif
