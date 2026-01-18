#include "physics/vehicle_dynamics.hpp"
#include "physics/tyre_model.hpp"

#include <algorithm>
#include <cmath>

namespace physics {
    static double interpolate (const std::vector<double>& x, const std::vector<double>& y, double xq) {
        if (xq <= x.front()) return y.front();
        if (xq >= x.back())  return y.back();

        for (size_t i = 1; i < x.size(); ++i) {
            if (xq < x[i]) {
                const double t = (xq - x[i-1]) / (x[i] - x[i-1]);
                return y[i-1] + t * (y[i] - y[i-1]);
            }
        }
        return y.back();
    }
}