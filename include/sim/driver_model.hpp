#pragma once

#include "models/car_state.hpp"
#include "models/control.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "sim/braking_envelope.hpp"

/**
 * @file driver_model.hpp
 * @brief Algorithmic driver that computes optimal control inputs.
 *
 * In a lap time simulation, the "driver" is an algorithm that decides
 * throttle and brake inputs to minimize lap time while respecting
 * physical limits (grip, engine power, etc.).
 *
 * This driver follows the car's braking envelope (BrakingEnvelope): the fastest
 * speed anywhere on the lap from which every corner ahead can still be made.
 *   1. Full throttle while that keeps the car under the envelope
 *   2. Otherwise exactly the throttle or brake that lands the car on it
 *   3. Select appropriate gear based on engine RPM
 *
 * Braking starts as late as the car's own brakes and tyres allow, and the
 * car reaches each corner at the corner's speed limit.
 *
 * More sophisticated models would include:
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
     *   2. Look up the braking envelope where the car will be after this step
     *   3. Full throttle if that stays under it; otherwise the throttle or
     *      brake that meets it
     *
     * @param state Current car state (position, velocity)
     * @param vehicle Vehicle parameters (for the available forces)
     * @param track Track definition (segments with curvature)
     * @param envelope The car's braking envelope on this track
     * @param dt Time step the inputs will be held for (s)
     * @return Control inputs (throttle, brake normalized to [0,1])
     */
    [[nodiscard]] ControlInput compute_control(
        const CarState& state,
        const VehicleParams& vehicle,
        const Track& track,
        const BrakingEnvelope& envelope,
        double dt) const;

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
};
