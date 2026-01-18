#pragma once

#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"

namespace physics {
    namespace lateral {
        /// Computes maximum allowable speed for a given track segment
        /// based on lateral acceleration limits.
        ///
        /// v_max = sqrt(mu * g / curvature)
        ///
        /// curvature == 0 implies straight (no lateral limit)

        double max_speed(
            const TrackSegment& segment,
            const VehicleParams& vehicle,
            const CarState& state
            );
    } // namespace physics
} // namespace lateral
