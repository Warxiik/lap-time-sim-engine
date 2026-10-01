#include "sim/step.hpp"
#include "physics/vehicle_dynamics.hpp"

#include <cmath>
#include <algorithm>

namespace step {

/**
 * @brief Advances the simulation state by one timestep.
 *
 * This is the main simulation step function: it calls the physics
 * integrator in the segment the car is in (the simulator finds it, once per
 * step, with its SegmentIndex).
 *
 * === No Speed Clamp ===
 *
 * The speed is never cut to the corner limit. The tyres limit what the
 * engine and brakes can do (the friction ellipse in vehicle_dynamics.cpp),
 * and the driver model brakes early enough to reach each corner at its
 * limit. A car pushed into a corner too fast keeps the speed it has: it has
 * no grip left to brake with there, just as a real car would run wide.
 *
 * === Integration ===
 *
 * The actual physics integration is delegated to vehicle_dynamics.cpp.
 * This separation keeps the step function clean and focused on
 * orchestration rather than physics details.
 *
 * @param state Car state to update (modified in place)
 * @param vehicle Vehicle parameters
 * @param segment The track segment the car is in
 * @param control Driver inputs (throttle, brake)
 * @param dt Time step (seconds)
 */
void advance(CarState& state,
             const VehicleParams& vehicle,
             const TrackSegment& segment,
             const ControlInput& control,
             seconds dt) {

    // Delegate to physics integrator for force calculation and state update
    physics::step_longitudinal(
        state,
        vehicle,
        segment,
        control.throttle,
        control.brake,
        dt
    );

    // Ensure velocity doesn't go negative (no reversing in this model)
    state.v = std::max(0.0, state.v);
}

} // namespace step
