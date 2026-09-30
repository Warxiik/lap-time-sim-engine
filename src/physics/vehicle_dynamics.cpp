#include "physics/vehicle_dynamics.hpp"
#include "physics/aero.hpp"
#include "physics/longitudinal.hpp"
#include "physics/tyre_model.hpp"
#include "core/constants.hpp"
#include "models/control.hpp"

#include <algorithm>
#include <cmath>

namespace physics {

/**
 * @brief Computes engine RPM from wheel speed and current gear.
 *
 * The relationship between engine RPM and wheel speed is:
 *   wheel_angular_velocity = v / r_wheel (rad/s)
 *   engine_angular_velocity = wheel_angular_velocity * gear_ratio * final_drive
 *   RPM = engine_angular_velocity * 60 / (2π)
 *
 * Combining: RPM = (v / r_wheel) * gear_ratio * final_drive * 60 / (2π)
 *                = v * gear_ratio * final_drive * 60 / (2π * r_wheel)
 *
 * @param velocity Current vehicle speed (m/s)
 * @param gear Current gear (1-indexed)
 * @param vehicle Vehicle parameters
 * @return Engine RPM
 */
static double compute_rpm(double velocity,
                          int gear,
                          const VehicleParams& vehicle) {
    const Gearbox& gearbox = vehicle.drivetrain.gearbox;

    // Validate gear
    const int gear_index = gear - 1;
    if (gear_index < 0 || gear_index >= static_cast<int>(gearbox.ratios.size())) {
        return 0.0;
    }

    // Angular velocity of wheels (rad/s)
    const double wheel_omega = velocity / vehicle.wheel_radius;

    // Engine angular velocity (rad/s) = wheel omega * total gear ratio
    const double gear_ratio = gearbox.ratios[gear_index];
    const double engine_omega = wheel_omega * gear_ratio * gearbox.final_drive;

    // Convert to RPM: ω (rad/s) * 60 / (2π) = ω * 9.5493
    constexpr double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * M_PI);
    return engine_omega * RAD_PER_SEC_TO_RPM;
}

/**
 * @brief The longitudinal forces available in the current state.
 *
 * The engine and the brakes say what they could deliver; the tyres say how
 * much of it reaches the road. The tyres' share is the friction ellipse's
 * (longitudinal_grip_force): all of the longitudinal grip on a straight,
 * less in a corner, none at the cornering limit.
 *
 * @param state Current car state (speed, gear, RPM)
 * @param vehicle Vehicle parameters
 * @param trackSeg Current track segment (grip, curvature, camber)
 * @return Forces in Newtons
 */
LongitudinalForces longitudinal_forces(const CarState& state,
                                       const VehicleParams& vehicle,
                                       const TrackSegment& trackSeg) {
    LongitudinalForces forces{};
    forces.engine = longitudinal::max_drive_force(vehicle, state);
    forces.brakes = longitudinal::max_brake_force(vehicle, state);
    forces.grip = longitudinal_grip_force(state, vehicle, trackSeg);
    forces.drag = aero::drag_force(vehicle, state);
    return forces;
}

/**
 * @brief Force the tyres transmit for the driver's pedals.
 *
 * The engine pushes and the brakes hold back; whatever is asked beyond the
 * grip is lost to wheelspin or lock-up, so the result is clamped to ±grip.
 */
double tyre_force(const LongitudinalForces& forces, double throttle, double brake) {
    const double asked = throttle * forces.engine - brake * forces.brakes;
    return std::clamp(asked, -forces.grip, forces.grip);
}

/**
 * @brief Advances the car state by one time step using longitudinal dynamics.
 *
 * This is the core simulation step function. It:
 *   1. Computes all forces acting on the vehicle
 *   2. Applies Newton's second law (F = ma → a = F/m)
 *   3. Integrates to update velocity and position
 *   4. Updates engine state (RPM)
 *
 * === Integration Method ===
 *
 * We use semi-implicit Euler integration:
 *   a(t) = F(t) / m
 *   v(t+dt) = v(t) + a(t) * dt
 *   s(t+dt) = s(t) + v(t+dt) * dt   ← Note: uses NEW velocity
 *
 * This is more stable than basic Euler and preserves energy better,
 * which is important for deterministic simulations.
 *
 * === Traction Limiting ===
 *
 * Both the drive force and the brake force are limited by what the tyres
 * can transmit along the road: μ_long times the normal load, times the
 * share the friction ellipse leaves after cornering. Nothing clamps the
 * speed: a car that arrives at a corner too fast has no grip left to brake
 * with, and runs through it at the speed it has. The driver model brakes
 * in time so that does not happen.
 *
 * === Assumptions ===
 *
 * This simplified model assumes:
 *   - Point mass (no weight transfer)
 *   - Perfect gear engagement (no clutch slip)
 *   - Constant drivetrain efficiency
 *   - No engine speed limiting (rev limiter)
 *
 * @param state Car state to update (modified in place)
 * @param vehicle Vehicle parameters
 * @param trackSeg Current track segment (for grip and curvature)
 * @param throttle Normalized throttle input [0, 1]
 * @param brake Normalized brake input [0, 1]
 * @param dt Time step in seconds
 */
void step_longitudinal(CarState& state,
                       const VehicleParams& vehicle,
                       const TrackSegment& trackSeg,
                       double throttle,
                       double brake,
                       seconds dt) {

    // === Step 1: Forces available (engine, brakes, tyre grip, drag) ===
    const LongitudinalForces forces = longitudinal_forces(state, vehicle, trackSeg);

    // === Step 2: What the tyres transmit, then drag ===
    // Drag is an external force, not limited by tyre grip
    const double net_force = tyre_force(forces, throttle, brake) - forces.drag;

    // === Step 3: Apply Newton's second law ===
    // F = ma → a = F/m
    const double acceleration = net_force / vehicle.mass;

    // === Step 4: Semi-implicit Euler integration ===
    // Update velocity first, then use new velocity for position
    // This is more stable than explicit Euler
    const double new_velocity = state.v + acceleration * dt;

    // Clamp velocity to non-negative (car can't go backwards in this model)
    state.v = std::max(0.0, new_velocity);

    // Update position using new velocity (semi-implicit)
    state.s += state.v * dt;

    // Store acceleration for telemetry/debugging
    state.a = acceleration;

    // === Step 5: Update engine RPM ===
    // RPM is determined by wheel speed and gear ratio
    state.engine_rpm = compute_rpm(state.v, state.gear, vehicle);
}

} // namespace physics
