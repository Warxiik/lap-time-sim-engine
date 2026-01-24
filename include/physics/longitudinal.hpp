#pragma once

#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/control.hpp"


namespace physics::longitudinal {
    /// Computes maximum available drive force (N)
    double max_drive_force(
        const VehicleParams& vehicle,
        const CarState& car_state
    );

    /// Computes maximum available braking force (N)
    double max_brake_force(
        const VehicleParams& vehicle,
        const CarState& car_state
    );

    /// Computes net longitudinal force applied to the vehicle (N)
    double net_force(
        const VehicleParams& vehicle,
        const CarState& car_state,
        const ControlInput& control,
        double drag_force
    );

} // namespace physics::longitudinal
