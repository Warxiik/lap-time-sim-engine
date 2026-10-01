#pragma once

#include <vector>

#include "models/vehicle.hpp"

/**
 * @file gear_map.hpp
 * @brief The gear the driver is in at each speed, worked out once per car.
 *
 * DriverModel::select_gear finds the gear with the most drive force by trying
 * every gear. The gear depends on the speed alone, so the speeds where it
 * changes (the shift speeds) can be found once: the map samples the speeds up
 * to past the top gear's last RPM, and bisects every change it finds down to
 * the exact speed. A step then only looks the speed up among a handful of
 * shift speeds.
 */
class GearMap {
public:
    explicit GearMap(const VehicleParams& vehicle);

    /// The gear at `velocity` m/s: DriverModel::select_gear's.
    [[nodiscard]] int gear_at(double velocity) const;

    /// The speeds (m/s, ascending) at and above which each gear after the first takes over.
    [[nodiscard]] const std::vector<double>& shift_speeds() const { return shift_speeds_; }

private:
    std::vector<double> shift_speeds_;
    std::vector<int> gears_;  // gears_[0] below the first shift speed, gears_[k] from shift_speeds_[k - 1]
};
