/**
    A version of Chris Engelsma's polynomial regression implementation with
    miscellaneous optimizations made.
    https://gist.github.com/chrisengelsma/108f7ab0a746323beaaf7d6634cf4add
*/
#include <math.h>
#include "polyreg.h"

PolynomialRegression::PolynomialRegression(size_t pointCount, int order)
{
    N = pointCount;
    n = order;
    np1 = n + 1;
    np2 = n + 2;
    tnp1 = 2 * n + 1;

    X = vector<float>(tnp1);
    Y = vector<float>(np1);
    a = vector<float>(np1);
    B = vector<vector<float>>(np1, vector<float>(np2, 0));
}

void PolynomialRegression::polyreg(const vector<float> &x,
    const vector<float> &y, vector<float> &coeffs)
{
    // X = vector that stores values of sigma(xi^2n)
    for (int i = 0; i < tnp1; i++)
    {
        X[i] = 0;
        for (int j = 0; j < N; j++)
            X[i] += pow(x[j], i);
    }

    // B = normal augmented matrix that stores the equations
    for (int i = 0; i <= n; i++)
        for (int j = 0; j <= n; j++)
            B[i][j] = X[i + j];

    // Y = vector to store values of sigma(xi^n * yi)
    for (int i = 0; i < np1; i++)
    {
        Y[i] = 0.0;
        for (int j = 0; j < N; j++)
            Y[i] += pow(x[j], i) * y[j];
    }

    // Load values of Y into last column of B
    for (int i = 0; i <= n; i++)
        B[i][np1] = Y[i];

    // Pivoting of the B matrix
    for (int i = 0; i < np1; i++)
        for (int k = i + 1; k < np1; k++)
            if (B[i][i] < B[k][i])
                for (int j = 0; j <= np1; j++)
                {
                    tmp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = tmp;
                }

    // Perform Gaussian elimination
    // (1) Make all elements below the pivot equals to zero
    //     or eliminate the variable.
    for (int i = 0; i < n; i++)
        for (int k = i + 1; k < np1; k++)
        {
            float t = B[k][i] / B[i][i];
            for (int j = 0; j <= np1; j++)
                B[k][j] -= t * B[i][j]; // (1)
        }

    // Back substitution
    // (1) Set the variable as the rhs of last equation
    // (2) Subtract all lhs values except the target coefficient
    // (3) Divide rhs by coefficient of variable being calculated
    for (int i = n; i >= 0; i--)
    {
        a[i] = B[i][np1]; // (1)
        for (int j = 0; j < np1; j++)
            if (j != i)
                a[i] -= B[i][j] * a[j]; // (2)
        a[i] /= B[i][i]; // (3)
    }

    coeffs.resize(a.size());
    for (size_t i = 0; i < a.size(); i++)
        coeffs[i] = a[i];
}
