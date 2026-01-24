#include "sim/step.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "physics/lateral.hpp"

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
 *   2. Enforces the cornering speed limit
 *   3. Calls the physics integrator to update state
 *
 * === Speed Limiting ===
 *
 * Before integrating, we check if current speed exceeds the lateral
 * grip limit for the current corner. If so, we clamp to that limit.
 * This represents the physical reality that you can't corner faster
 * than grip allows — the car would slide off track.
 *
 * In a more sophisticated model, exceeding the limit would trigger
 * understeer/oversteer dynamics. For our point-mass model, we simply
 * enforce the limit as a hard constraint.
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

    // Calculate maximum cornering speed for this segment
    const double v_max_lateral = physics::lateral::max_speed(segment, vehicle, state);

    // Enforce cornering speed limit
    // This is a simplification — in reality, exceeding grip causes sliding
    // For our point-mass model, we treat it as a hard constraint
    if (state.v > v_max_lateral) {
        state.v = v_max_lateral;
    }

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

    // Re-check lateral limit after integration
    // (velocity may have increased beyond what the corner allows)
    const double v_max_after = physics::lateral::max_speed(segment, vehicle, state);
    if (state.v > v_max_after) {
        state.v = v_max_after;
    }
}

} // namespace step
