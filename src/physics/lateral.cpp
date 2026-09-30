#include "physics/lateral.hpp"
#include "core/constants.hpp"

#include <cmath>
#include <limits>

namespace physics::lateral {

/**
 * @brief Computes the maximum speed for a given track segment based on lateral grip.
 *
 * This is one of the most important functions in a lap time simulation.
 * It determines how fast the car can go through corners.
 *
 * === The Physics ===
 *
 * When a car corners, it needs centripetal force to change direction.
 * This force comes from tyre friction. If you go too fast, the required
 * centripetal force exceeds available grip, and you slide off the track.
 *
 * Centripetal acceleration: a_lat = v² · κ   (κ = curvature = 1/radius)
 *
 * The tyres can provide μ times the normal load, and the normal load grows
 * with downforce, which grows with v². So the limit has to be found at the
 * corner's own speed, not the speed the car happens to arrive with. On a
 * flat road the balance is
 *
 *   v² · κ = μ · (g + q · v²),      q = ρ · |Cl| · A / (2 m)
 *
 * which solves in closed form:
 *
 *   v² = μ · g / (κ − μ · q)
 *
 * === Banking ===
 *
 * A road banked by θ into the turn tips part of the weight into the
 * cornering direction and part of the cornering load into the road:
 *
 *   v²·κ·cos θ − g·sin θ = μ · (g·cos θ + v²·κ·sin θ + q·v²)
 *
 *   v² = g · (μ·cos θ + sin θ) / (κ · (cos θ − μ·sin θ) − μ · q)
 *
 * With θ = 0 this is the flat-road formula.
 *
 * If the denominator is not positive, the grip grows at least as fast as
 * the demand: no speed is too fast for this corner, and the car is limited
 * by power and drag instead.
 *
 * === Curvature Sign Convention ===
 *
 *   - κ = 0: straight line (no lateral limit)
 *   - κ > 0: left turn, κ < 0: right turn
 *
 * We use the absolute value since direction doesn't affect the speed limit.
 * The camber is measured into the turn whichever way it goes.
 *
 * @param segment Track segment with curvature, grip and camber
 * @param vehicle Vehicle parameters (mass, aero, tyres)
 * @return Maximum steady cornering speed in m/s
 */
double max_speed(const TrackSegment& segment,
                 const VehicleParams& vehicle) {

    const double curvature = std::abs(segment.curvature);

    // Straight section: no lateral acceleration limit
    if (curvature < 1e-9) {
        return std::numeric_limits<double>::max();
    }

    const double mu = vehicle.tyre.base_grip * segment.grip;
    // Downforce per unit mass and per v² (negative for a car with lift)
    const double q = 0.5 * constants::air_density * (-vehicle.aero.lift_coefficient) * vehicle.aero.frontal_area / vehicle.mass;
    const double cos_bank = std::cos(segment.camber);
    const double sin_bank = std::sin(segment.camber);

    const double numerator = constants::g * (mu * cos_bank + sin_bank);
    const double denominator = curvature * (cos_bank - mu * sin_bank) - mu * q;

    if (denominator <= 0.0) {
        return std::numeric_limits<double>::max();
    }
    if (numerator <= 0.0) {
        return 0.0;
    }

    return std::sqrt(numerator / denominator);
}

double max_speed(const TrackSegment& segment,
                 const VehicleParams& vehicle,
                 const CarState& /* state */) {
    return max_speed(segment, vehicle);
}

} // namespace physics::lateral
