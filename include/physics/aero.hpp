#pragma once

#include "models/vehicle.hpp"
#include "models/car_state.hpp"

namespace physics {
    namespace aero {
        /// Computes aerodynamic drag force (N)
        double drag_force (
            const VehicleParams& vehicle,
            const CarState& state
        );

        /// Computes aerodynamic downforce (N)
        double downforce (
            const VehicleParams& vehicle,
            const CarState& state
        );
    } // namespace aero
} // namespace physics