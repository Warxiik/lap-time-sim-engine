#pragma once

#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"

namespace physics {
    namespace lateral {
        /// Maximum steady speed through a track segment, where the lateral grip
        /// (with the downforce at that same speed, and the banking) just holds
        /// the corner:
        ///
        /// v_max = sqrt(g (μ cos θ + sin θ) / (|κ| (cos θ − μ sin θ) − μ ρ |Cl| A / (2 m)))
        ///
        /// curvature == 0 implies straight (no lateral limit), and so does a
        /// car whose downforce grows its grip faster than the corner's demand.
        double max_speed(
            const TrackSegment& segment,
            const VehicleParams& vehicle
            );

        /// As above. The car state is not needed: the corner speed does not
        /// depend on the speed the car arrives with.
        double max_speed(
            const TrackSegment& segment,
            const VehicleParams& vehicle,
            const CarState& state
            );
    } // namespace lateral
} // namespace physics
