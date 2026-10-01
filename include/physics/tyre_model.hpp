#pragma once

#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/track.hpp"

namespace physics {

    /// Acceleration pressing the car into the road (m/s²): gravity across the banking, the share of the
    /// cornering acceleration that banking turns into load, and downforce.
    double normal_acceleration(double velocity, const VehicleParams& vehicle, const TrackSegment& trackSeg);

    /// Lateral acceleration the tyres must provide along the road surface to follow the segment (m/s², >= 0).
    double lateral_demand(double velocity, const TrackSegment& trackSeg);

    /// Maximum lateral acceleration the tyres can provide (m/s²).
    double compute_lateral_acc_limit(const CarState& state, const VehicleParams& vehicle, const TrackSegment& trackSeg);

    /// Fraction of the longitudinal grip left after cornering, from the friction ellipse [0..1].
    double compute_traction_scale(const CarState& state, const VehicleParams& vehicle, const TrackSegment& trackSeg);

    /// Largest force the tyres can transmit along the road for driving or braking now (N).
    double longitudinal_grip_force(const CarState& state, const VehicleParams& vehicle, const TrackSegment& trackSeg);

    /// As above, with the friction ellipse's share already worked out (compute_traction_scale).
    double longitudinal_grip_force(const CarState& state, const VehicleParams& vehicle, const TrackSegment& trackSeg,
                                   double traction_scale);

} // namespace physics
