#include "utils.h"

#include <cmath>

// Read: https://www.wikiwand.com/en/Simpson%27s_rule (Composite Simpson's Rule)
double lengthIntegral(double t0, double t1, std::function<Eigen::Vector2d (double)> df, int n) {
    double dx = t1 - t0,
           f0 = hypot(df(t0)(0, 0), df(t1)(1, 0)),
           fn = hypot(df(t1)(0, 0), df(t1)(1, 0));

    double sum1, sum2;

    for (int k = 1; k < (n / 2); ++k) {
        double xk = t0 + (2 * k * dx) / n;
        Eigen::Vector2d der = df(xk);
        sum1 += hypot(der(0, 0), der(1, 0));
    }

    for (int k = 1; k < (n / 2) + 1; ++k) {
        double xk1 = t0 + (dx * (2 * k - 1)) / n;
        Eigen::Vector2d der = df(xk1);
        sum2 += hypot(der(0, 0), der(1, 0));
    }

    sum1 *= 2; sum2 *= 4;

    return (dx / (3 * n)) * (f0 + sum1 + sum2 + fn);
}

double meterToRotations(double m, double r, double g) {
    return g * m / (2 * PI * r);
}

double meterToNativeUnits(double m, double r, double nuPerRot, double g) {
    return nuPerRot * g * m / (2 * PI * r);
}

double mpsToRpm(double m, double r, double g) {
   return 60 * g * m / (2 * PI * r);
}

double mpsToNuPer100ms(double m, double r, double g) {
    
}
