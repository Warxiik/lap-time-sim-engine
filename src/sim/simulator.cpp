#include "sim/simulator.hpp"

/**
 * @brief Constructs a simulator with track, vehicle, and configuration.
 *
 * The simulator takes ownership of track and vehicle data via move semantics.
 * This avoids unnecessary copies of potentially large data structures.
 *
 * @param track Track definition (moved)
 * @param vehicle Vehicle parameters (moved)
 * @param config Simulation configuration (timestep, limits)
 */
Simulator::Simulator(Track track, VehicleParams vehicle, SimConfig config)
    : track_(std::move(track)),
      vehicle_(std::move(vehicle)),
      config_(config),
      state_(),
      telemetry_(),
      driver_()
{
    // Initialize car state at start of track, stationary
    state_.s = 0.0;           // Start at beginning of track
    state_.v = 0.1;           // Small initial velocity to avoid division issues
    state_.a = 0.0;
    state_.gear = 1;          // Start in first gear
    state_.engine_rpm = 0.0;  // Will be calculated on first step

    // Pre-allocate telemetry based on estimated lap time
    // Rough estimate: track_length / 50 m/s average speed / dt
    const double estimated_time = track_.total_length / 50.0;
    const size_t estimated_frames = static_cast<size_t>(estimated_time / config_.dt) + 1000;
    telemetry_.reserve(estimated_frames);
}

/**
 * @brief Runs the simulation until the car completes the track.
 *
 * Main simulation loop:
 *   1. Compute driver inputs based on current state
 *   2. Update gear selection
 *   3. Advance physics by one timestep
 *   4. Record telemetry
 *   5. Repeat until track completed or time limit reached
 *
 * The simulation uses a fixed timestep for determinism. Variable timesteps
 * can introduce subtle numerical differences that accumulate over a lap.
 */
void Simulator::run() {
    const seconds dt = config_.dt;
    seconds elapsed_time = 0.0;

    // Safety limit to prevent infinite loops
    const seconds max_time = config_.max_time;

    // Clear any previous telemetry data
    telemetry_.clear();

    while (state_.s < track_.total_length && elapsed_time < max_time) {
        // Step 1: Driver decides throttle/brake based on current situation
        const ControlInput control = driver_.compute_control(state_, vehicle_, track_);

        // Step 2: Update gear selection based on RPM
        state_.gear = driver_.select_gear(state_, vehicle_);

        // Step 3: Advance physics simulation
        step::advance(state_, vehicle_, track_, control, dt);

        // Step 4: Record telemetry for analysis
        telemetry_.record(state_, control, elapsed_time);

        // Step 5: Advance time
        elapsed_time += dt;
    }
}

/**
 * @brief Returns the recorded telemetry data.
 *
 * @return Const reference to telemetry (avoid copying large data)
 */
const Telemetry& Simulator::telemetry() const {
    return telemetry_;
}

/**
 * @brief Returns the total lap time.
 *
 * The lap time is the final recorded timestamp in telemetry.
 * Returns 0 if no data has been recorded.
 *
 * @return Lap time in seconds
 */
seconds Simulator::lap_time() const {
    if (telemetry_.time.empty()) {
        return 0.0;
    }
    return telemetry_.time.back();
}
