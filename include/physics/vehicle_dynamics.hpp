#pragma once

#include "core/units.hpp"
#include "models/vehicle.hpp"
#include "models/track.hpp"
#include "models/car_state.hpp"

namespace physics {

    /// The longitudinal forces available to the car in its current state (N, all >= 0).
    struct LongitudinalForces {
        double engine;  // at the wheels at full throttle in the current gear
        double brakes;  // at full brake
        double grip;    // what the tyres can transmit along the road (friction ellipse)
        double drag;    // aerodynamic drag
    };

    LongitudinalForces longitudinal_forces(
        const CarState& state,
        const VehicleParams& vehicle,
        const TrackSegment& trackSeg
    );

    /// Force the tyres transmit for these pedals: the engine's minus the brakes', within the grip (N, + forward).
    double tyre_force(const LongitudinalForces& forces, double throttle, double brake);

    void step_longitudinal(
        CarState& state,
        const VehicleParams& vehicle,
        const TrackSegment& trackSeg,
        double throttle, // normalized [0..1]
        double brake, // normalized [0..1]
        seconds dt
    );
}
