#include "sim/step.hpp"
#include "physics/vehicle_dynamics.hpp"

#include <cmath>
#include <algorithm>

namespace step {

/**
 * @brief Finds the track segment containing the given position.
 *
 * @param track Track definition
 * @param position Distance along track (m)
 * @return Reference to the segment at that position
 */
static const TrackSegment& find_segment(const Track& track, double position) {
    // Handle positions beyond track length (wrap around)
    const double wrapped_pos = std::fmod(position, track.total_length);

    double accumulated = 0.0;
    for (const auto& seg : track.segments) {
        accumulated += seg.length;
        if (wrapped_pos < accumulated) {
            return seg;
        }
    }

    // Fallback to last segment
    return track.segments.back();
}

/**
 * @brief Advances the simulation state by one timestep.
 *
 * This is the main simulation step function that:
 *   1. Finds the current track segment
 *   2. Calls the physics integrator to update state
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
 * @param track Track definition
 * @param control Driver inputs (throttle, brake)
 * @param dt Time step (seconds)
 */
void advance(CarState& state,
             const VehicleParams& vehicle,
             const Track& track,
             const ControlInput& control,
             seconds dt) {

    // Find current track segment based on position
    const TrackSegment& segment = find_segment(track, state.s);

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
