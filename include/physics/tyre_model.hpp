#pragma once

#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/track.hpp"

namespace physics {
    double compute_lateral_acc_limit(const CarState& state, const VehicleParams& vehicle, const TrackSegment& trackSeg);
    double compute_traction_scale(const CarState& state, const VehicleParams& vehicle, const TrackSegment& trackSeg);
} // namespace physics