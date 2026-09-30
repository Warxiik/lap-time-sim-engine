#include "physics/longitudinal.hpp"
#include "physics/aero.hpp"
#include "physics/tyre_model.hpp"
#include "core/constants.hpp"
#include "core/math.hpp"

#include <algorithm>
#include <cmath>

namespace physics::longitudinal {

/**
 * @brief Computes the maximum drive force available at the wheels.
 *
 * This function models the powertrain from engine to wheels:
 *
 *   Engine → Gearbox → Final Drive → Wheels
 *
 * The force calculation follows this chain:
 *   1. Look up engine torque at current RPM
 *   2. Multiply by current gear ratio (mechanical advantage)
 *   3. Multiply by final drive ratio (differential)
 *   4. Apply drivetrain efficiency (losses in transmission)
 *   5. Divide by wheel radius to convert torque to force
 *
 * Formula: F = (T_engine * gear_ratio * final_drive * efficiency) / r_wheel
 *
 * Physical interpretation:
 *   - Higher gear ratio = more torque multiplication but lower top speed
 *   - Lower gear ratio = less torque but higher wheel speed
 *   - This is why cars accelerate faster in 1st gear than 8th
 *
 * @param vehicle Vehicle parameters (drivetrain, wheel radius)
 * @param car_state Current state (RPM, gear selection)
 * @return Maximum drive force in Newtons
 */
double max_drive_force(const VehicleParams& vehicle, const CarState& car_state) {
    const Drivetrain& dt = vehicle.drivetrain;
    const Engine& engine = dt.engine;
    const Gearbox& gearbox = dt.gearbox;

    // Safety check: valid gear selection
    // Gears are 1-indexed in real cars, but 0-indexed in our array
    const int gear_index = car_state.gear - 1;
    if (gear_index < 0 || gear_index >= static_cast<int>(gearbox.ratios.size())) {
        return 0.0;  // Invalid gear, no drive force
    }

    // Step 1: Get engine torque at current RPM from the torque curve
    const double engine_torque = math::interpolate(engine.rpm, engine.torque, car_state.engine_rpm);

    // Step 2-4: Apply gear ratios and efficiency
    const double gear_ratio = gearbox.ratios[gear_index];
    const double wheel_torque = engine_torque * gear_ratio * gearbox.final_drive * dt.efficiency;

    // Step 5: Convert torque at wheel to force (F = T / r)
    const double drive_force = wheel_torque / vehicle.wheel_radius;

    // Clamp to vehicle's maximum (accounts for structural/traction limits)
    return std::min(drive_force, vehicle.max_drive_force);
}

/**
 * @brief Computes the maximum braking force available.
 *
 * In a simplified model, we treat brakes as capable of producing up to
 * a maximum retarding force. In reality, braking is limited by:
 *   - Brake disc/caliper capacity (thermal and mechanical limits)
 *   - Tire grip (can't brake harder than tires can hold)
 *
 * For our point-mass model, we use a single max_brake_force value that
 * represents the system limit. Tire grip limiting is handled separately
 * via the traction circle calculation.
 *
 * @param vehicle Vehicle parameters (max brake force)
 * @param car_state Current state (unused in simplified model, but available
 *                  for future enhancements like brake fade)
 * @return Maximum braking force in Newtons (positive value)
 */
double max_brake_force(const VehicleParams& vehicle, const CarState& /* car_state */) {
    // In this simplified model, braking force is constant
    // Future enhancement: model brake fade (reduced braking after sustained use)
    // Future enhancement: speed-dependent aero braking contribution
    return vehicle.max_brake_force;
}

/**
 * @brief Computes the net longitudinal force on the vehicle.
 *
 * This is the key function that determines acceleration or deceleration.
 * It combines all longitudinal forces:
 *
 *   F_net = F_drive - F_brake - F_drag
 *
 * Where:
 *   F_drive = throttle * max_drive_force (positive, forward)
 *   F_brake = brake * max_brake_force (subtractive, resisting motion)
 *   F_drag  = aerodynamic drag (always opposes motion)
 *
 * Sign convention:
 *   - Positive = accelerating (forward force)
 *   - Negative = decelerating (rearward force)
 *
 * Note on traction limits:
 *   The available traction for longitudinal forces depends on how much
 *   grip is being used for cornering. This is the "friction circle" concept:
 *   the tire has a total grip budget shared between lateral and longitudinal.
 *   This is handled by compute_traction_scale() in the tyre model.
 *
 * @param vehicle Vehicle parameters
 * @param car_state Current car state
 * @param control Driver inputs (throttle/brake)
 * @param drag Aerodynamic drag force (passed in to avoid recalculation)
 * @return Net longitudinal force in Newtons
 */
double net_force(const VehicleParams& vehicle,
                 const CarState& car_state,
                 const ControlInput& control,
                 double drag) {
    // Calculate available drive and brake forces
    const double available_drive = max_drive_force(vehicle, car_state);
    const double available_brake = max_brake_force(vehicle, car_state);

    // Apply driver inputs (throttle and brake are normalized 0-1)
    const double drive_force = control.throttle * available_drive;
    const double brake_force = control.brake * available_brake;

    // Net force: drive pushes forward, brake and drag push backward
    // Note: drag is already a positive value representing resistance
    return drive_force - brake_force - drag;
}

} // namespace physics::longitudinal
