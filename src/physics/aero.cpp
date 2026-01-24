#include "physics/aero.hpp"
#include "core/constants.hpp"

namespace physics::aero {

/**
 * @brief Computes aerodynamic drag force acting on the vehicle.
 *
 * Drag is the resistive force that opposes the car's motion through air.
 * It increases with the square of velocity, which is why top speed is limited
 * even with unlimited power - at some point drag equals available thrust.
 *
 * Formula: F_drag = 0.5 * ρ * Cd * A * v²
 *
 * Where:
 *   ρ  = air density (kg/m³), typically 1.225 at sea level
 *   Cd = drag coefficient (dimensionless), depends on car shape
 *   A  = frontal area (m²), the cross-sectional area facing the wind
 *   v  = velocity (m/s)
 *
 * Typical F1 values:
 *   Cd ≈ 0.7-1.0 (high due to wings and open wheels)
 *   A  ≈ 1.5 m²
 *
 * @param vehicle Vehicle parameters containing aero coefficients
 * @param state Current car state (velocity is used)
 * @return Drag force in Newtons (always positive, opposes motion)
 */
double drag_force(const VehicleParams& vehicle, const CarState& state) {
    const Aero& aero = vehicle.aero;
    const double v_squared = state.v * state.v;

    // Standard aerodynamic drag equation
    // The 0.5 factor comes from the kinetic energy formulation in fluid dynamics
    return 0.5 * constants::air_density
               * aero.drag_coefficient
               * aero.frontal_area
               * v_squared;
}

/**
 * @brief Computes aerodynamic downforce acting on the vehicle.
 *
 * Downforce is the vertical aerodynamic load that pushes the car into the ground.
 * This is critical for racing because it increases tire grip without adding mass.
 * More grip means higher cornering speeds and better acceleration/braking.
 *
 * Formula: F_downforce = 0.5 * ρ * |Cl| * A * v²
 *
 * Where:
 *   ρ  = air density (kg/m³)
 *   Cl = lift coefficient (negative for downforce by convention)
 *   A  = reference area (m²)
 *   v  = velocity (m/s)
 *
 * Sign convention:
 *   - In aerodynamics, positive Cl means upward lift (like airplane wings)
 *   - Race cars have negative Cl because wings are inverted to push DOWN
 *   - This function returns POSITIVE downforce (magnitude of the downward force)
 *
 * Typical F1 values:
 *   Cl ≈ -3.0 to -4.0 (very high downforce)
 *   At 300 km/h, an F1 car generates ~3x its weight in downforce
 *
 * @param vehicle Vehicle parameters containing aero coefficients
 * @param state Current car state (velocity is used)
 * @return Downforce in Newtons (positive value, pushes car into ground)
 */
double downforce(const VehicleParams& vehicle, const CarState& state) {
    const Aero& aero = vehicle.aero;
    const double v_squared = state.v * state.v;

    // Lift coefficient is negative for downforce (inverted wing)
    // We negate it to return a positive downforce value
    // If someone incorrectly uses positive Cl, this returns negative (lift),
    // which will reduce grip - a natural consequence of the physics
    const double cl_magnitude = -aero.lift_coefficient;

    return 0.5 * constants::air_density
               * cl_magnitude
               * aero.frontal_area
               * v_squared;
}

} // namespace physics::aero
