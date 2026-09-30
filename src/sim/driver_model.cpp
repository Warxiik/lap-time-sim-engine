#include "sim/driver_model.hpp"
#include "physics/vehicle_dynamics.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

/**
 * @brief Finds the track segment index for a given track position.
 *
 * Track segments are laid out sequentially. We iterate through and
 * accumulate length until we find the segment containing our position.
 *
 * Time complexity: O(n) where n = number of segments
 * For performance-critical code, this could be optimized with binary search
 * or a lookup table, but for typical track sizes (~100 segments) this is fine.
 */
size_t DriverModel::find_segment_index(const Track& track, double position) const {
    // Handle wraparound for positions beyond track length
    const double wrapped_pos = std::fmod(position, track.total_length);

    double accumulated = 0.0;
    for (size_t i = 0; i < track.segments.size(); ++i) {
        accumulated += track.segments[i].length;
        if (wrapped_pos < accumulated) {
            return i;
        }
    }

    // Should not reach here if track is properly defined
    return track.segments.size() - 1;
}

/**
 * @brief Simple gear selection based on engine RPM.
 *
 * Strategy:
 *   - Shift up when RPM exceeds 90% of max (approaching rev limiter)
 *   - Shift down when RPM drops below 40% of max (lugging the engine)
 *
 * Real F1 cars use much more sophisticated logic considering:
 *   - Torque curves and power bands
 *   - Upcoming track features
 *   - Tire slip optimization
 *   - Energy recovery modes
 */
int DriverModel::select_gear(const CarState& state,
                             const VehicleParams& vehicle) const {
    const auto& engine = vehicle.drivetrain.engine;
    const auto& gearbox = vehicle.drivetrain.gearbox;
    const int num_gears = static_cast<int>(gearbox.ratios.size());

    // Get RPM range from engine data
    const double max_rpm = engine.rpm.back();
    const double min_rpm = engine.rpm.front();

    // Shift thresholds
    const double upshift_rpm = max_rpm * 0.90;    // Shift up at 90% of redline
    const double downshift_rpm = max_rpm * 0.40;  // Shift down at 40% of redline

    int current_gear = state.gear;

    // Clamp to valid range
    current_gear = std::clamp(current_gear, 1, num_gears);

    // Check for upshift
    if (state.engine_rpm > upshift_rpm && current_gear < num_gears) {
        return current_gear + 1;
    }

    // Check for downshift
    if (state.engine_rpm < downshift_rpm && current_gear > 1) {
        return current_gear - 1;
    }

    return current_gear;
}

/**
 * @brief Main driver logic: decide throttle and brake for current state.
 *
 * Algorithm overview:
 *   1. Find the speed the braking envelope allows where the car will be
 *      after this step (at its current speed, v * dt further on)
 *   2. If full throttle keeps the car under it: full throttle
 *   3. Otherwise: the tyre force that lands the car exactly on it, as a
 *      fraction of the engine's force (throttle) or the brakes' (brake)
 *
 * The envelope already holds every corner's limit and the braking distance
 * to it, computed with the same forces the physics step applies, so the
 * inputs never need to be more than full brake. Braking eases off as the
 * corner's limit comes up, and in a corner at its limit the throttle is
 * open but the tyres have no grip left to transmit it.
 */
ControlInput DriverModel::compute_control(const CarState& state,
                                          const VehicleParams& vehicle,
                                          const Track& track,
                                          const BrakingEnvelope& envelope,
                                          double dt) const {
    const TrackSegment& segment = track.segments[find_segment_index(track, state.s)];
    const physics::LongitudinalForces forces = physics::longitudinal_forces(state, vehicle, segment);

    // Where the envelope stands after this step
    const double allowed = envelope.speed_at(state.s + state.v * dt);

    // Full throttle, if that stays under it
    const double full_throttle_accel = (physics::tyre_force(forces, 1.0, 0.0) - forces.drag) / vehicle.mass;
    if (state.v + full_throttle_accel * dt <= allowed) {
        return ControlInput{1.0, 0.0};
    }

    // Otherwise the tyre force that meets it: F = m (v_allowed - v) / dt + drag
    const double needed = vehicle.mass * (allowed - state.v) / dt + forces.drag;
    if (needed >= 0.0) {
        const double throttle = forces.engine > 0.0 ? std::min(1.0, needed / forces.engine) : 0.0;
        return ControlInput{throttle, 0.0};
    }
    const double brake = forces.brakes > 0.0 ? std::min(1.0, -needed / forces.brakes) : 1.0;
    return ControlInput{0.0, brake};
}
