#include "sim/step.hpp"
#include "physics/lateral.hpp"
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
 * === Over the Corner's Limit ===
 *
 * The tyres limit what the engine and brakes can do (the friction ellipse in
 * vehicle_dynamics.cpp), and the driver model brakes early enough to reach
 * each corner at its limit, so a car that follows its braking envelope is
 * never cut back.
 *
 * A car that reaches a corner faster than its limit anyway loses the excess
 * there: a point mass cannot run wide, and keeping the speed would let it go
 * through the corner faster than its tyres allow. A car falls behind its
 * envelope when its grip drops under braking (the tyres' condition changes
 * the envelope while the car is on full brake already), and in a braking
 * zone through a bend the more it is behind, the more of its grip cornering
 * takes and the less is left to brake with. Carrying the speed through then
 * made such a car the quicker the further behind it fell, and a stint's
 * laps jumped by half a second between two branches at a kink in a braking
 * zone.
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

    // Over the corner's limit: the excess is lost (above)
    state.v = std::min(state.v, physics::lateral::max_speed(segment, vehicle));
}

} // namespace step
