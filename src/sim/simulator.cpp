#include "sim/simulator.hpp"
#include "core/math.hpp"
#include "physics/tyre_condition.hpp"
#include "physics/vehicle_dynamics.hpp"

#include <algorithm>
#include <cmath>

/**
 * @brief Constructs a simulator with track, vehicle, and configuration.
 *
 * The simulator takes ownership of track and vehicle data via move semantics.
 * This avoids unnecessary copies of potentially large data structures.
 *
 * The braking envelope the driver follows, the segment index and the shift
 * speeds are computed here, once: they depend only on the track and the car.
 * A car that burns fuel or wears its tyres (VehicleParams::fuel,
 * VehicleParams::tyre_condition) gets its envelope planned again as it changes.
 *
 * @param track Track definition (moved)
 * @param vehicle Vehicle parameters (moved)
 * @param config Simulation configuration (timestep, limits, flying lap, laps)
 */
Simulator::Simulator(Track track, VehicleParams vehicle, SimConfig config)
    : track_(std::move(track)),
      vehicle_(std::move(vehicle)),
      config_(config),
      current_(vehicle_),
      envelope_(compute_braking_envelope(track_, vehicle_, config_.envelope_spacing)),
      index_(track_),
      gears_(vehicle_),
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

    // The stint's starting fuel and fresh tyres
    state_.fuel = vehicle_.fuel.mass;
    start_mass_ = vehicle_.mass + vehicle_.fuel.mass;
    tyre_steps_ = std::max(1, static_cast<int>(std::lround(0.01 / config_.dt)));
    if (vehicle_.tyre_condition.enabled) {
        const TyreState front = physics::tyres::fresh(vehicle_.tyre_condition.front);
        const TyreState rear = physics::tyres::fresh(vehicle_.tyre_condition.rear);
        state_.tyres = {front, front, rear, rear};
        felt_grip_ = std::min(front.grip, rear.grip);
    }
    if (consumables()) {
        update_vehicle();
        replan();
    }

    // Pre-allocate telemetry based on estimated lap time
    // Rough estimate: track_length / 50 m/s average speed / dt
    const double estimated_time = track_.total_length / 50.0 * std::max(1, config_.laps);
    const size_t estimated_frames = static_cast<size_t>(estimated_time / config_.dt) + 1000;
    telemetry_.reserve(estimated_frames);
}

bool Simulator::consumables() const {
    return vehicle_.fuel.mass > 0.0 || vehicle_.tyre_condition.enabled;
}

/**
 * @brief The car as it is now.
 *
 * Its mass is the car's with the fuel left. Its friction is the tyres', times
 * the weakest tyre's condition for cornering and braking (the driver drives to
 * the tyre that would let go first), and its drive's traction limit follows the
 * driven axle's condition and the car's weight.
 */
double Simulator::car_grip() const {
    return vehicle_.tyre_condition.enabled ? felt_grip_ : 1.0;
}

void Simulator::update_vehicle() {
    current_.mass = vehicle_.mass + state_.fuel;
    double drive = current_.mass / start_mass_;
    if (vehicle_.tyre_condition.enabled) {
        const double both = car_grip();
        const std::size_t first = vehicle_.tyre_condition.front_driven ? TyreIndex::front_left : TyreIndex::rear_left;
        const double driven = 0.5 * (state_.tyres[first].grip + state_.tyres[first + 1].grip);
        current_.tyre.base_grip = vehicle_.tyre.base_grip * both;
        current_.tyre.longitudinal_grip = vehicle_.tyre.longitudinal_grip * both;
        drive *= driven;
    }
    current_.max_drive_force = vehicle_.max_drive_force * drive;
}

void Simulator::replan() {
    envelope_ = compute_braking_envelope(track_, current_, config_.envelope_spacing);
    envelope_grip_ = car_grip();
    envelope_scale_ = 1.0;
}

/**
 * @brief What a step cost in fuel: brake-specific consumption on the crank's
 * work (the throttle's share of the full-load torque, times the engine speed)
 * plus the idle flow.
 */
void Simulator::burn(const CarState& before, const ControlInput& control, seconds dt) {
    if (vehicle_.fuel.bsfc > 0.0 && state_.fuel > 0.0) {
        const Engine& engine = current_.drivetrain.engine;
        const double torque = math::interpolate(engine.rpm, engine.torque, before.engine_rpm);
        const double crank_power = std::max(0.0, control.throttle * torque * before.engine_rpm * 2.0 * M_PI / 60.0);
        const double burnt = (vehicle_.fuel.bsfc * crank_power / 3.6e9 + vehicle_.fuel.idle_flow) * dt;
        state_.fuel = std::max(0.0, state_.fuel - burnt);
    }
}

/**
 * @brief What `dt` of driving costs the tyres: each axle's load and forces and
 * how they split across it (physics::tyres::split), the sliding work the grip
 * they use implies, and each tyre's two thermal nodes, pressure and wear
 * (physics::tyres::advance). Then the weakest tyre's grip, as the driver feels
 * it over TyreConditionParams::feel_time.
 *
 * The tyres' temperatures move over seconds, so they are updated every 10 ms,
 * from the forces of that step, rather than every step.
 */
void Simulator::wear_tyres(const CarState& before, const TrackSegment& segment, double tyre_force, seconds dt) {
    const TyreConditionParams& condition = vehicle_.tyre_condition;
    physics::tyres::AxleWork front;
    physics::tyres::AxleWork rear;
    physics::tyres::split(condition, current_, current_.mass, before.v, tyre_force, segment, front, rear);
    // The grip used is the share of the tyres' own friction (TyreParams' unless the axle says otherwise).
    const auto own = [](double axle, double car) { return axle > 0.0 ? axle : car; };
    front.mu_long = own(condition.front.mu_longitudinal, vehicle_.tyre.longitudinal_grip) * segment.grip;
    front.mu_lat = own(condition.front.mu_lateral, vehicle_.tyre.base_grip) * segment.grip;
    rear.mu_long = own(condition.rear.mu_longitudinal, vehicle_.tyre.longitudinal_grip) * segment.grip;
    rear.mu_lat = own(condition.rear.mu_lateral, vehicle_.tyre.base_grip) * segment.grip;
    physics::tyres::advance(condition.front, condition, front, before.v, dt, state_.tyres[TyreIndex::front_left], state_.tyres[TyreIndex::front_right]);
    physics::tyres::advance(condition.rear, condition, rear, before.v, dt, state_.tyres[TyreIndex::rear_left], state_.tyres[TyreIndex::rear_right]);

    double weakest = 1.0;
    for (const TyreState& t : state_.tyres) weakest = std::min(weakest, t.grip);
    felt_grip_ = condition.feel_time > 0.0 ? felt_grip_ + (weakest - felt_grip_) * std::min(1.0, dt / condition.feel_time)
                                            : weakest;
}

/**
 * @brief Advances the car by one step: gear, driver inputs, physics.
 *
 * The gear comes first, with the engine speed it gives, so the driver's
 * inputs and the step use the same forces.
 */
ControlInput Simulator::step_once(seconds dt) {
    const TrackSegment& segment = track_.segments[index_.find(state_.s)];

    // The best gear for the speed the car has (DriverModel::select_gear's, from the shift speeds)
    state_.gear = gears_.gear_at(state_.v);
    state_.engine_rpm = physics::engine_rpm(state_.v, state_.gear, current_);

    // Driver decides throttle/brake from the braking envelope
    const ControlInput control = driver_.compute_control(state_, current_, segment, envelope_, dt, envelope_scale_);

    if (!consumables()) {
        step::advance(state_, current_, segment, control, dt);
        return control;
    }

    // What the step costs in fuel, and every 10 ms in tyre (from the force the tyres put through the road)
    const CarState before = state_;
    const bool tyres = vehicle_.tyre_condition.enabled && --tyre_countdown_ <= 0;
    const double tyre_force =
        tyres ? physics::tyre_force(physics::longitudinal_forces(before, current_, segment), control.throttle, control.brake) : 0.0;
    step::advance(state_, current_, segment, control, dt);
    burn(before, control, dt);
    if (tyres) {
        wear_tyres(before, segment, tyre_force, tyre_steps_ * dt);
        tyre_countdown_ = tyre_steps_;
    }
    update_vehicle();

    // Between plans the driver takes the envelope's speeds with the grip the tyres have now: corner speeds
    // and braking distances go with the square root of the grip, as a driver feels it. Once the grip has
    // moved 2 % from what the envelope was planned with, it is planned again.
    if (vehicle_.tyre_condition.enabled) {
        const double ratio = car_grip() / envelope_grip_;
        if (std::abs(ratio - 1.0) > 0.02) {
            replan();
        } else {
            envelope_scale_ = std::sqrt(ratio);
        }
    }
    return control;
}

/**
 * @brief Runs the simulation until the car completes its laps.
 *
 * Main simulation loop:
 *   1. Select the gear for the current speed
 *   2. Compute driver inputs based on current state
 *   3. Advance physics by one timestep (and the fuel and tyres)
 *   4. Record telemetry
 *   5. Repeat until the laps are completed or the time limit reached
 *
 * === Standing and flying laps ===
 *
 * By default the lap starts from rest at the line. With
 * SimConfig::flying_lap the car first drives an untimed out lap, and the
 * timed lap starts as it crosses the line at the speed it carries there,
 * as a qualifying lap does. On a closed circuit that is the speed of any
 * lap after the first.
 *
 * === Stints ===
 *
 * With SimConfig::laps above 1 the car drives on after the line, lap after
 * lap, each timed line to line (LapRecord), carrying its fuel and tyres. The
 * braking envelope is planned again at each line for the car as it is then.
 *
 * === Timing ===
 *
 * Each telemetry frame is recorded with the time of the state it holds,
 * i.e. after its step. Each lap time is interpolated within the step that
 * crosses the line (and, on a flying lap, the step that crossed it at the
 * start), so it does not depend on where the steps happen to fall.
 *
 * The simulation uses a fixed timestep for determinism. Variable timesteps
 * can introduce subtle numerical differences that accumulate over a lap.
 */
void Simulator::run() {
    const seconds dt = config_.dt;
    const double length = track_.total_length;
    const int laps = std::max(1, config_.laps);

    // Safety limit to prevent infinite loops (the out lap counts)
    const seconds max_time = config_.max_time;
    seconds total_time = 0.0;

    // Clear any previous telemetry data
    telemetry_.clear();
    laps_.clear();
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
        if (consumables()) replan();
        telemetry_.record(state_, control, elapsed_time);
    }

    int lap = 0;
    seconds lap_start = 0.0;
    LapRecord record;
    record.fuel_at_start = state_.fuel;
    while (lap < laps && total_time < max_time) {
        const double before = state_.s;

        // Driver, gear, physics
        const ControlInput control = step_once(dt);

        // Advance time, then record the state it reached (distance running on from lap to lap)
        elapsed_time += dt;
        total_time += dt;
        if (lap == 0) {
            telemetry_.record(state_, control, elapsed_time);
        } else {
            CarState frame = state_;
            frame.s += lap * length;
            telemetry_.record(frame, control, elapsed_time);
        }

        if (state_.s >= length) {
            // Crossed the line within this step
            const seconds crossing = elapsed_time - dt + (length - before) / (state_.s - before) * dt;
            record.time = crossing - lap_start;
            record.fuel_at_end = state_.fuel;
            record.tyres = state_.tyres;
            record.front = physics::tyres::average(state_.tyres[TyreIndex::front_left], state_.tyres[TyreIndex::front_right]);
            record.rear = physics::tyres::average(state_.tyres[TyreIndex::rear_left], state_.tyres[TyreIndex::rear_right]);
            laps_.push_back(record);
            lap_start = crossing;
            ++lap;
            if (lap < laps) {
                state_.s -= length;
                record = LapRecord{};
                record.fuel_at_start = state_.fuel;
                if (consumables()) replan();
            }
        }
    }
    completed_ = lap == laps;
    lap_time_ = laps_.empty() ? elapsed_time : laps_.front().time;
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
 * @return Lap time in seconds: line to line on the first finished lap, else
 *         the time driven
 */
seconds Simulator::lap_time() const {
    return lap_time_;
}

bool Simulator::completed() const {
    return completed_;
}

const std::vector<LapRecord>& Simulator::laps() const {
    return laps_;
}

const BrakingEnvelope& Simulator::braking_envelope() const {
    return envelope_;
}

const VehicleParams& Simulator::vehicle_now() const {
    return current_;
}

const CarState& Simulator::state() const {
    return state_;
}
