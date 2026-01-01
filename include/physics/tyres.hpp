#pragma once

#include "models/vehicle.hpp"
#include "models/car_state.hpp"

namespace physics {
    namespace tyres {
        /// Returns effective grip coefficient μ
        double grip_coefficient (
            const VehicleParams& vehicle,
            const CarState& state,
            double normal_load
            );

    } // namespace tyres
} // namespace physics