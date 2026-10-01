#include "sim/gear_map.hpp"

#include <algorithm>
#include <cmath>

#include "models/car_state.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "sim/driver_model.hpp"

namespace {

int best_gear(const VehicleParams& vehicle, double velocity) {
    CarState state{};
    state.v = velocity;
    return DriverModel{}.select_gear(state, vehicle);
}

}  // namespace

GearMap::GearMap(const VehicleParams& vehicle) {
    const auto& ratios = vehicle.drivetrain.gearbox.ratios;
    const int gears = static_cast<int>(ratios.size());
    gears_.push_back(best_gear(vehicle, 0.0));
    if (gears == 0 || vehicle.drivetrain.engine.rpm.empty()) {
        return;
    }

    // Past the top gear's last RPM every gear is over the curve: top gear from there on.
    const double rpm_per_mps = physics::engine_rpm(1.0, gears, vehicle);
    const double top = rpm_per_mps > 0.0 ? 1.01 * vehicle.drivetrain.engine.rpm.back() / rpm_per_mps : 0.0;
    constexpr double kStep = 0.05;  // m/s between samples: a gear holds for much more than this
    const int samples = static_cast<int>(std::ceil(top / kStep));

    double low = 0.0;
    int gear = gears_.front();
    for (int k = 1; k <= samples; ++k) {
        const double high = k * kStep;
        const int next = best_gear(vehicle, high);
        if (next != gear) {
            // The change lies in (low, high]: bisect to the first speed of the new gear.
            double a = low;
            double b = high;
            for (int i = 0; i < 60; ++i) {
                const double mid = 0.5 * (a + b);
                if (best_gear(vehicle, mid) == gear) {
                    a = mid;
                } else {
                    b = mid;
                }
            }
            shift_speeds_.push_back(b);
            gears_.push_back(next);
            gear = next;
        }
        low = high;
    }
    if (gear != gears) {
        shift_speeds_.push_back(top);
        gears_.push_back(gears);
    }
}

int GearMap::gear_at(double velocity) const {
    const auto it = std::upper_bound(shift_speeds_.begin(), shift_speeds_.end(), velocity);
    return gears_[static_cast<size_t>(it - shift_speeds_.begin())];
}
