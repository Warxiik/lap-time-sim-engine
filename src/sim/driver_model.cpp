#include "sim/driver_model.hpp"
#include "physics/lateral.hpp"
#include "core/constants.hpp"

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
 * @brief Computes distance needed to brake from current speed to target.
 *
 * Derivation from kinematics:
 *   v² = v₀² + 2*a*d
 *   d = (v² - v₀²) / (2*a)
 *
 * Since we're decelerating, a is negative, but we pass deceleration as
 * a positive value, so:
 *   d = (v₀² - v²) / (2*decel)
 *
 * We add a safety margin (10%) to account for:
 *   - Discrete timestep effects
 *   - Grip variations
 *   - Conservative driving
 */
double DriverModel::braking_distance(double current_speed,
                                     double target_speed,
                                     double deceleration) const {
    if (current_speed <= target_speed) {
        return 0.0;  // No braking needed
    }

    if (deceleration <= 0.0) {
        return std::numeric_limits<double>::max();  // Can't brake
    }

    const double v0_sq = current_speed * current_speed;
    const double v_sq = target_speed * target_speed;

    // Base braking distance from kinematics
    const double base_distance = (v0_sq - v_sq) / (2.0 * deceleration);

    // Add 10% safety margin
    constexpr double SAFETY_MARGIN = 1.1;
    return base_distance * SAFETY_MARGIN;
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
 *   1. Find current segment and look ahead to upcoming segments
 *   2. Calculate maximum safe speed for each upcoming segment
 *   3. Determine if we need to brake to make the corner
 *   4. If braking not needed, apply full throttle
 *
 * This is a simplified "bang-bang" controller (full throttle or full brake).
 * More sophisticated models would use:
 *   - Proportional control for smoother inputs
 *   - Trail braking (partial brake while turning)
 *   - Throttle modulation for traction control
 */
ControlInput DriverModel::compute_control(const CarState& state,
                                          const VehicleParams& vehicle,
                                          const Track& track) const {
    ControlInput control{0.0, 0.0};

    // Find current position on track
    const size_t current_idx = find_segment_index(track, state.s);
    const TrackSegment& current_seg = track.segments[current_idx];

    // Maximum speed for current segment (lateral grip limit)
    const double v_max_current = physics::lateral::max_speed(current_seg, vehicle, state);

    // Estimate braking deceleration (assume ~1.5g for F1-like car)
    // In a more complete model, this would come from tire/aero calculations
    constexpr double ASSUMED_DECEL = 1.5 * constants::g;  // ~14.7 m/s²

    // Look ahead: find minimum speed in upcoming segments
    // and check if we need to start braking
    double distance_to_brake_point = 0.0;
    double min_upcoming_speed = v_max_current;

    // Accumulate distance through current segment
    double pos_in_segment = state.s;
    for (size_t i = 0; i < current_idx; ++i) {
        pos_in_segment -= track.segments[i].length;
    }
    distance_to_brake_point = current_seg.length - pos_in_segment;

    // Look ahead through upcoming segments (limit lookahead to avoid excessive computation)
    constexpr size_t MAX_LOOKAHEAD = 10;
    const size_t num_segments = track.segments.size();

    for (size_t i = 1; i <= MAX_LOOKAHEAD; ++i) {
        const size_t seg_idx = (current_idx + i) % num_segments;
        const TrackSegment& seg = track.segments[seg_idx];

        // Calculate max speed for this segment
        // Note: Using current state for downforce, which is approximate
        // A more accurate model would iterate to find consistent speed
        const double v_max_seg = physics::lateral::max_speed(seg, vehicle, state);

        // Check if we need to brake for this corner
        const double brake_dist = braking_distance(state.v, v_max_seg, ASSUMED_DECEL);

        if (brake_dist > distance_to_brake_point) {
            // Need to start braking now!
            control.throttle = 0.0;
            control.brake = 1.0;
            return control;
        }

        // Track minimum speed and accumulate distance
        min_upcoming_speed = std::min(min_upcoming_speed, v_max_seg);
        distance_to_brake_point += seg.length;
    }

    // No braking needed - check if we're below corner speed limit
    if (state.v < v_max_current * 0.99) {
        // Below limit: accelerate
        control.throttle = 1.0;
        control.brake = 0.0;
    } else {
        // At limit: maintain speed (coast)
        // In reality you'd use partial throttle to maintain exact speed
        control.throttle = 0.3;  // Light throttle to maintain
        control.brake = 0.0;
    }

    return control;
}
