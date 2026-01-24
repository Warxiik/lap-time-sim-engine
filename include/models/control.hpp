#pragma once

/**
 * @file control.hpp
 * @brief Driver control inputs for the vehicle simulation.
 *
 * In a lap time simulation, the "driver" is typically an algorithm that decides
 * how much throttle and brake to apply based on the car's state and upcoming track.
 * This struct represents those normalized control commands.
 */

/**
 * @struct ControlInput
 * @brief Represents the driver's control commands at a given instant.
 *
 * All inputs are normalized to [0, 1] range:
 * - 0.0 = no input (pedal fully released)
 * - 1.0 = full input (pedal fully pressed)
 *
 * In real racing, throttle and brake can be applied simultaneously (left-foot braking),
 * but for a simplified point-mass model, we typically use one or the other.
 */
struct ControlInput {
    double throttle;  // Normalized throttle position [0..1]
    double brake;     // Normalized brake position [0..1]
};
