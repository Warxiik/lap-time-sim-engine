#include "sim/driver_model.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "core/math.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

/**
 * @brief The gear with the most drive force at the car's speed.
 *
 * Every gear's engine speed follows from the car's (engine_rpm); the drive
 * force in a gear is the torque there times the gear's ratio (the final drive,
 * the efficiency and the wheel radius are the same in every gear). Gears that
 * would take the engine past its torque curve's last RPM are left out unless
 * none is left, and of the rest the strongest wins (the lower gear on a tie).
 *
 * Shifting at RPM thresholds instead makes the gear depend on the gear before
 * it: a car leaving a corner just above the downshift RPM stays a gear too
 * high down the whole straight after it, just below it drops a gear, and a
 * few kilograms decide which.
 */
int DriverModel::select_gear(const CarState& state,
                             const VehicleParams& vehicle) const {
    const auto& engine = vehicle.drivetrain.engine;
    const auto& ratios = vehicle.drivetrain.gearbox.ratios;
    const int num_gears = static_cast<int>(ratios.size());
    if (num_gears == 0 || engine.rpm.empty()) {
        return 1;
    }

    const double max_rpm = engine.rpm.back();
    int best = num_gears;
    double best_force = -std::numeric_limits<double>::infinity();
    for (int gear = 1; gear <= num_gears; ++gear) {
        const double rpm = physics::engine_rpm(state.v, gear, vehicle);
        if (rpm > max_rpm) {
            continue;  // past the curve: a higher gear
        }
        const double force = math::interpolate(engine.rpm, engine.torque, rpm) * ratios[static_cast<size_t>(gear - 1)];
        if (force > best_force) {
            best_force = force;
            best = gear;
        }
    }
    return best;
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
                                          const TrackSegment& segment,
                                          const BrakingEnvelope& envelope,
                                          double dt) const {
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
