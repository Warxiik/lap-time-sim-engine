#pragma once

#include "models/car_state.hpp"
#include "models/control.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "physics/lateral.hpp"

/**
 * @file driver_model.hpp
 * @brief Algorithmic driver that computes optimal control inputs.
 *
 * In a lap time simulation, the "driver" is an algorithm that decides
 * throttle and brake inputs to minimize lap time while respecting
 * physical limits (grip, engine power, etc.).
 *
 * This simplified driver model uses a basic strategy:
 *   1. Look ahead to find the maximum safe speed for upcoming corners
 *   2. If current speed > safe speed: brake
 *   3. If current speed < safe speed: accelerate
 *   4. Select appropriate gear based on engine RPM
 *
 * More sophisticated models would include:
 *   - Trail braking (brake while turning)
 *   - Optimal racing line selection
 *   - Tire management (adjusting pace to preserve tires)
 *   - Energy deployment strategy (for hybrid systems)
 */

/**
 * @class DriverModel
 * @brief Computes control inputs to drive the car around the track.
 *
 * The driver model is stateless in this implementation — it makes
 * decisions purely based on current state and track information.
 * This ensures deterministic behavior.
 */
class DriverModel {
public:
    /**
     * @brief Computes throttle and brake inputs for current situation.
     *
     * The algorithm:
     *   1. Find current track segment based on position
     *   2. Compute maximum speed for current and upcoming segments
     *   3. Determine if we need to brake or can accelerate
     *   4. Return appropriate control inputs
     *
     * @param state Current car state (position, velocity)
     * @param vehicle Vehicle parameters (for speed calculations)
     * @param track Track definition (segments with curvature)
     * @return Control inputs (throttle, brake normalized to [0,1])
     */
    [[nodiscard]] ControlInput compute_control(
        const CarState& state,
        const VehicleParams& vehicle,
        const Track& track) const;

    /**
     * @brief Determines optimal gear for current speed and RPM.
     *
     * Simple algorithm: shift up if RPM exceeds threshold,
     * shift down if RPM drops below threshold.
     *
     * @param state Current car state
     * @param vehicle Vehicle parameters (gear ratios, RPM limits)
     * @return Recommended gear (1-indexed)
     */
    [[nodiscard]] int select_gear(
        const CarState& state,
        const VehicleParams& vehicle) const;

private:
    /**
     * @brief Finds the track segment containing the given position.
     *
     * @param track Track definition
     * @param position Distance along track (m)
     * @return Index of segment containing position
     */
    [[nodiscard]] size_t find_segment_index(
        const Track& track,
        double position) const;

    /**
     * @brief Computes braking distance from current speed to target speed.
     *
     * Uses kinematic equation: d = (v² - v_target²) / (2 * a_brake)
     *
     * @param current_speed Current velocity (m/s)
     * @param target_speed Target velocity (m/s)
     * @param deceleration Braking deceleration (m/s², positive value)
     * @return Distance required to slow down (m)
     */
    [[nodiscard]] double braking_distance(
        double current_speed,
        double target_speed,
        double deceleration) const;
};
