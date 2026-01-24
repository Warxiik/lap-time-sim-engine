#pragma once

#include <vector>
#include "models/car_state.hpp"
#include "models/control.hpp"

/**
 * @file telemetry.hpp
 * @brief Data recording for simulation analysis.
 *
 * Telemetry captures the time history of the simulation, storing
 * state and control data at each timestep. This enables:
 *   - Post-simulation analysis and visualization
 *   - Debugging physics behavior
 *   - Comparing different setups or strategies
 *   - Generating lap time breakdowns
 *
 * In real F1, telemetry is transmitted live to the pit wall and factory,
 * capturing hundreds of channels at high frequency. Our simulation
 * telemetry serves a similar purpose for analysis.
 */

/**
 * @struct TelemetryFrame
 * @brief A single snapshot of simulation state at one instant.
 *
 * Each frame captures the complete state of the car and driver inputs
 * at a specific point in time. Frames are stored sequentially to
 * build a complete time history.
 */
struct TelemetryFrame {
    double time;        // Simulation time (s)
    double distance;    // Distance along track (m)
    double velocity;    // Speed (m/s)
    double acceleration;// Longitudinal acceleration (m/s²)
    int gear;           // Current gear
    double engine_rpm;  // Engine speed (RPM)
    double throttle;    // Throttle input [0,1]
    double brake;       // Brake input [0,1]
};

/**
 * @struct Telemetry
 * @brief Complete recording of a simulation run.
 *
 * Stores parallel vectors for each telemetry channel. This structure
 * is cache-friendly for iterating over single channels (e.g., plotting
 * velocity vs distance) while still allowing frame-by-frame analysis.
 *
 * Design note: We use separate vectors rather than vector<TelemetryFrame>
 * because:
 *   1. Better cache locality when analyzing single channels
 *   2. Easier to extend with new channels
 *   3. More efficient for large datasets (no padding)
 */
struct Telemetry {
    // Time-series data vectors (all same length)
    std::vector<double> time;           // Elapsed time (s)
    std::vector<double> distance;       // Track position (m)
    std::vector<double> velocity;       // Speed (m/s)
    std::vector<double> acceleration;   // Longitudinal accel (m/s²)
    std::vector<int> gear;              // Gear number
    std::vector<double> engine_rpm;     // Engine RPM
    std::vector<double> throttle;       // Throttle position [0,1]
    std::vector<double> brake;          // Brake position [0,1]

    /**
     * @brief Records current state and control to telemetry.
     *
     * Call this once per simulation step to build the time history.
     * The elapsed_time should be tracked externally and passed in.
     *
     * @param state Current car state
     * @param control Current driver inputs
     * @param elapsed_time Time since simulation start (s)
     */
    void record(const CarState& state, const ControlInput& control, double elapsed_time) {
        time.push_back(elapsed_time);
        distance.push_back(state.s);
        velocity.push_back(state.v);
        acceleration.push_back(state.a);
        gear.push_back(state.gear);
        engine_rpm.push_back(state.engine_rpm);
        throttle.push_back(control.throttle);
        brake.push_back(control.brake);
    }

    /**
     * @brief Pre-allocates memory for expected number of frames.
     *
     * Call this before simulation to avoid repeated reallocations.
     * Estimate frames as: (track_length / estimated_avg_speed) / dt
     *
     * Example: 5km track, 50 m/s average, 1ms timestep = 100,000 frames
     *
     * This is important for:
     *   - Performance: avoids O(log n) reallocations during recording
     *   - Determinism: removes allocation timing variability
     *   - Memory efficiency: single allocation vs fragmented growth
     *
     * @param expected_frames Estimated number of simulation steps
     */
    void reserve(size_t expected_frames) {
        time.reserve(expected_frames);
        distance.reserve(expected_frames);
        velocity.reserve(expected_frames);
        acceleration.reserve(expected_frames);
        gear.reserve(expected_frames);
        engine_rpm.reserve(expected_frames);
        throttle.reserve(expected_frames);
        brake.reserve(expected_frames);
    }

    /**
     * @brief Clears all recorded data.
     *
     * Use before starting a new simulation run.
     * Note: This does NOT release memory. Use shrink_to_fit() after
     * clear() if you need to release memory.
     */
    void clear() {
        time.clear();
        distance.clear();
        velocity.clear();
        acceleration.clear();
        gear.clear();
        engine_rpm.clear();
        throttle.clear();
        brake.clear();
    }

    /**
     * @brief Returns the number of recorded frames.
     */
    [[nodiscard]] size_t size() const {
        return time.size();
    }
};
