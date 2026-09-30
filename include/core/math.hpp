#pragma once

#include <cstddef>
#include <vector>

namespace math {

/**
 * @brief Linear interpolation in a table of points (x sorted ascending).
 *
 * Used for curves given as points, such as engine torque against RPM. A query
 * outside the table takes the nearest end's value rather than extrapolating:
 * an engine does not make more torque past the last point it was measured at.
 *
 * @param x  Sorted x values (e.g. RPM)
 * @param y  The y value at each x (e.g. torque)
 * @param xq Query point
 * @return y at xq, clamped to the table's ends
 */
inline double interpolate(const std::vector<double>& x,
                          const std::vector<double>& y,
                          double xq) {
    if (xq <= x.front()) return y.front();
    if (xq >= x.back())  return y.back();

    for (std::size_t i = 1; i < x.size(); ++i) {
        if (xq < x[i]) {
            // y = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
            const double t = (xq - x[i - 1]) / (x[i] - x[i - 1]);
            return y[i - 1] + t * (y[i] - y[i - 1]);
        }
    }
    return y.back();
}

} // namespace math
