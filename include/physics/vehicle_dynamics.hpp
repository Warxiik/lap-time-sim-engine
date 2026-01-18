#pragma once

#include "core/units.hpp"
#include "models/vehicle.hpp"
#include "models/track.hpp"
#include "models/car_state.hpp"

namespace physics {
    void step_longitudinal(
        CarState& state,
        const VehicleParams& vehicle,
        const TrackSegment& trackSeg,
        double throttle, // normalized [0..1]
        double brake, // normalized [0..1]
        seconds dt
    );
}