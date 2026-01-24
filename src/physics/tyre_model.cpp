#include "physics/tyre_model.hpp"
#include "physics/aero.hpp"
#include "core/constants.hpp"

#include <algorithm>
#include <cmath>

namespace physics {

/**
 * @brief Computes the maximum lateral acceleration the car can sustain.
 *
 * This function answers: "How hard can the car corner without losing grip?"
 *
 * === Fundamental Physics ===
 *
 * Friction force = μ * N
 *   Where μ = friction coefficient (grip), N = normal force
 *
 * For a car cornering:
 *   - Normal force N = weight + downforce = m*g + F_downforce
 *   - Maximum friction F_max = μ * N
 *   - Maximum lateral acceleration a_lat_max = F_max / m = μ * N / m
 *
 * Substituting:
 *   a_lat_max = μ * (m*g + F_downforce) / m
 *             = μ * g + μ * F_downforce / m
 *
 * === The Downforce Advantage ===
 *
 * Notice that downforce adds grip without adding to the denominator (mass).
 * This is the key insight behind aerodynamic grip in racing:
 *
 *   - Road car: a_lat_max ≈ μ * g ≈ 1.0 * 9.81 ≈ 10 m/s² ≈ 1g
 *   - F1 car at 200 km/h: downforce ≈ 2 * weight
 *     a_lat_max ≈ μ * 3g ≈ 1.5 * 3 * 9.81 ≈ 44 m/s² ≈ 4.5g
 *
 * This is why F1 cars can take corners that would be impossible for road cars.
 *
 * === Track Grip Factor ===
 *
 * Different track surfaces and conditions affect grip:
 *   - Dry asphalt: grip ≈ 1.0
 *   - Wet track: grip ≈ 0.7
 *   - Gravel: grip ≈ 0.4
 *
 * We multiply the base μ by the track's grip factor.
 *
 * @param state Current car state (velocity affects downforce)
 * @param vehicle Vehicle parameters (mass, aero configuration)
 * @param trackSeg Track segment (local grip coefficient)
 * @return Maximum sustainable lateral acceleration in m/s²
 */
double compute_lateral_acc_limit(const CarState& state,
                                 const VehicleParams& vehicle,
                                 const TrackSegment& trackSeg) {
    // Calculate downforce at current speed using the aero module
    // This properly handles the sign convention (returns positive downforce)
    const double aero_downforce = aero::downforce(vehicle, state);

    // Total normal force = gravitational weight + aerodynamic downforce
    // Both are in Newtons, pushing the car into the track
    const double normal_force = vehicle.mass * constants::g + aero_downforce;

    // Effective grip coefficient = track surface grip
    // In a more complex model, this would also include tire compound effects,
    // temperature, wear, and camber angle effects
    const double mu = trackSeg.grip;

    // Maximum lateral acceleration = (μ * N) / m
    // This comes from: F_friction = μ * N, and a = F / m
    return mu * normal_force / vehicle.mass;
}

/**
 * @brief Computes the fraction of longitudinal traction available after cornering.
 *
 * This implements the "friction circle" (or "traction circle") concept,
 * which is fundamental to vehicle dynamics and racing.
 *
 * === The Friction Circle ===
 *
 * Tires have a limited total grip budget. This budget is shared between:
 *   - Lateral force (cornering)
 *   - Longitudinal force (acceleration/braking)
 *
 * Imagine a circle where:
 *   - The radius represents maximum tire grip
 *   - Horizontal axis = longitudinal force
 *   - Vertical axis = lateral force
 *
 * The tire can produce any combination of forces that stays INSIDE the circle.
 * If you're using 80% of grip for cornering, you only have ~60% left for braking.
 *
 * Mathematically (simplified linear model):
 *   traction_available = 1 - (a_lat_current / a_lat_max)
 *
 * More accurate (circular):
 *   sqrt(a_lat² + a_long²) ≤ a_max
 *   a_long_available = sqrt(a_max² - a_lat²)
 *
 * We use the linear model for simplicity. It's slightly pessimistic but stable.
 *
 * === Racing Application ===
 *
 * - On a straight: a_lat = 0, so full traction available for acceleration/braking
 * - In a fast corner: a_lat is high, less traction for throttle/brake
 * - At corner apex: often at grip limit, no room for throttle
 * - Trail braking: gradually releasing brake as you add steering
 *
 * This is why "smooth" driving is fast—jerky inputs exceed the friction circle.
 *
 * @param state Current car state (velocity for downforce, position for segment)
 * @param vehicle Vehicle parameters
 * @param trackSeg Track segment (curvature determines required lateral acceleration)
 * @return Fraction of longitudinal traction available [0.0, 1.0]
 */
double compute_traction_scale(const CarState& state,
                              const VehicleParams& vehicle,
                              const TrackSegment& trackSeg) {
    // Current lateral acceleration required for this corner at current speed
    // a_lat = v² * curvature (centripetal acceleration formula)
    const double a_lat_current = state.v * state.v * std::abs(trackSeg.curvature);

    // Maximum lateral acceleration available from tires
    const double a_lat_max = compute_lateral_acc_limit(state, vehicle, trackSeg);

    // Edge case: if no lateral grip (shouldn't happen), full traction available
    // This prevents division by zero and handles pathological inputs
    if (a_lat_max <= 0.0) {
        return 1.0;
    }

    // Calculate remaining traction budget
    // Linear friction circle: traction = 1 - (used_grip / total_grip)
    const double lateral_usage = a_lat_current / a_lat_max;
    const double scale = 1.0 - lateral_usage;

    // Clamp to valid range [0, 1]
    // - scale < 0 means we're already exceeding grip (car is sliding)
    // - scale > 1 shouldn't happen mathematically, but clamp for safety
    return std::clamp(scale, 0.0, 1.0);
}

} // namespace physics
