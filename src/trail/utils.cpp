#include "utils.h"

#include <cmath>

std::string cwd() {
    char temp[FILENAME_MAX];
    CWD(temp, FILENAME_MAX);
    return std::string(temp);
}

int readFile(const std::string &filename, std::string *content) {
    std::string line, total;
    std::ifstream file(filename);

    if (file.is_open()) {
        while (std::getline(file, line)) {
            total += line + "\n";
        }
        file.close();
        content->assign(total);
        return 0;
    } else {
        return -1;
    }
}

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

std::vector<double> lengthIntegrals(double t0, double t1, std::function<std::vector<Eigen::Vector2d> (double)> dfs, int n) {
    double dt = t1 - t0, sum1, sum2, tk, tk1, f0, fn;
    std::vector<Eigen::Vector2d> fs0 = dfs(t0), fsn = dfs(t1);
    std::vector<double> lengths;

    // O(shit) time complexity, O(fuck) memory complexity
    for (int i = 0; i < fs0.size(); ++i) {
        f0 = hypot(fs0[i](0, 0), fs0[i](1, 0)); fn = hypot(fsn[i](0, 0), fsn[i](1, 0));

        for (int k = 1; k < (n / 2); ++k) {
            tk = t0 + (2 * k * dt) / n;
            Eigen::Vector2d der = dfs(tk)[i];
            sum1 += hypot(der(0, 0), der(1, 0));
        }

        for (int k = 1; k < (n / 2) + 1; ++k) {
            tk1 = t0 + (dt * (2 * k - 1)) / n;
            Eigen::Vector2d der = dfs(tk1)[i];
            sum2 += hypot(der(0, 0), der(1, 0));
        }

        sum1 *= 2; sum2 *= 4;
        lengths.emplace_back((dt / (3 * n)) * (f0 + sum1 + sum2 + fn));
    }

    return lengths;
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
    return -3316.0; // TODO - Implement
}
