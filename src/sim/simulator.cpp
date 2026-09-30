#include "sim/simulator.hpp"

/**
 * @brief Constructs a simulator with track, vehicle, and configuration.
 *
 * The simulator takes ownership of track and vehicle data via move semantics.
 * This avoids unnecessary copies of potentially large data structures.
 *
 * The braking envelope the driver follows is computed here, once: it depends
 * only on the track and the car.
 *
 * @param track Track definition (moved)
 * @param vehicle Vehicle parameters (moved)
 * @param config Simulation configuration (timestep, limits, flying lap)
 */
Simulator::Simulator(Track track, VehicleParams vehicle, SimConfig config)
    : track_(std::move(track)),
      vehicle_(std::move(vehicle)),
      config_(config),
      envelope_(compute_braking_envelope(track_, vehicle_, config_.envelope_spacing)),
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
 * @brief Advances the car by one step: driver inputs, gear, physics.
 */
ControlInput Simulator::step_once(seconds dt) {
    // Driver decides throttle/brake from the braking envelope
    const ControlInput control = driver_.compute_control(state_, vehicle_, track_, envelope_, dt);

    // Update gear selection based on RPM
    state_.gear = driver_.select_gear(state_, vehicle_);

    // Advance physics simulation
    step::advance(state_, vehicle_, track_, control, dt);
    return control;
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
 * === Standing and flying laps ===
 *
 * By default the lap starts from rest at the line. With
 * SimConfig::flying_lap the car first drives an untimed out lap, and the
 * timed lap starts as it crosses the line at the speed it carries there,
 * as a qualifying lap does. On a closed circuit that is the speed of any
 * lap after the first.
 *
 * === Timing ===
 *
 * Each telemetry frame is recorded with the time of the state it holds,
 * i.e. after its step. The lap time is interpolated within the step that
 * crosses the line (and, on a flying lap, the step that crossed it at the
 * start), so it does not depend on where the steps happen to fall.
 *
 * The simulation uses a fixed timestep for determinism. Variable timesteps
 * can introduce subtle numerical differences that accumulate over a lap.
 */
void Simulator::run() {
    const seconds dt = config_.dt;
    const double length = track_.total_length;

    // Safety limit to prevent infinite loops (the out lap counts)
    const seconds max_time = config_.max_time;
    seconds total_time = 0.0;

    // Clear any previous telemetry data
    telemetry_.clear();
    completed_ = false;

    // The clock starts at the line: from rest, or part way through the step
    // that carried the car across it at the end of the out lap.
    seconds elapsed_time = 0.0;
    if (config_.flying_lap) {
        double before = state_.s;
        ControlInput control{};
        while (state_.s < length && total_time < max_time) {
            before = state_.s;
            control = step_once(dt);
            total_time += dt;
        }
        if (state_.s < length) {
            lap_time_ = 0.0;
            return;  // out of time on the out lap
        }
        // The first state past the line opens the timed lap.
        const double past_line = (state_.s - length) / (state_.s - before);
        state_.s -= length;
        elapsed_time = past_line * dt;
        telemetry_.record(state_, control, elapsed_time);
    }

    while (state_.s < length && total_time < max_time) {
        const double before = state_.s;

        // Driver, gear, physics
        const ControlInput control = step_once(dt);

        // Advance time, then record the state it reached
        elapsed_time += dt;
        total_time += dt;
        telemetry_.record(state_, control, elapsed_time);

        if (state_.s >= length) {
            // Crossed the line within this step
            completed_ = true;
            lap_time_ = elapsed_time - dt + (length - before) / (state_.s - before) * dt;
        }
    }
    if (!completed_) {
        lap_time_ = elapsed_time;
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
 * @brief Returns the lap time.
 *
 * @return Lap time in seconds: line to line on a finished lap, else the
 *         time driven
 */
seconds Simulator::lap_time() const {
    return lap_time_;
}

bool Simulator::completed() const {
    return completed_;
}

const BrakingEnvelope& Simulator::braking_envelope() const {
    return envelope_;
}
