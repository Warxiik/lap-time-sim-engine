#include "physics/tyre_model.hpp"
#include "physics/aero.hpp"
#include "core/constants.hpp"

#include <algorithm>
#include <cmath>

namespace physics {

/**
 * @brief Acceleration pressing the car into the road surface.
 *
 * On a road banked by θ into the turn (the segment's camber), the car's
 * weight and the centripetal acceleration it needs split between the
 * direction across the surface and the direction into it:
 *
 *   a_normal = g·cos θ + v²·|κ|·sin θ + F_downforce / m
 *
 * On a flat road (θ = 0) this is g plus the downforce per unit mass, the
 * load the friction coefficient acts on.
 *
 * @param velocity Speed (m/s)
 * @param vehicle Vehicle parameters (mass, aero)
 * @param trackSeg Track segment (curvature, camber)
 * @return Normal acceleration in m/s²
 */
double normal_acceleration(double velocity, const VehicleParams& vehicle, const TrackSegment& trackSeg) {
    CarState at_speed{};
    at_speed.v = velocity;
    const double downforce = aero::downforce(vehicle, at_speed);
    const double k = std::abs(trackSeg.curvature);
    return constants::g * std::cos(trackSeg.camber)
         + velocity * velocity * k * std::sin(trackSeg.camber)
         + downforce / vehicle.mass;
}

/**
 * @brief Lateral acceleration the tyres must provide along the surface.
 *
 *   a_lateral = | v²·|κ|·cos θ − g·sin θ |
 *
 * Banking takes part of the cornering load off the tyres (the g·sin θ term):
 * on a banked turn at low speed the tyres even hold the car from sliding
 * down the slope. On a flat road it is the centripetal acceleration v²·|κ|.
 *
 * @param velocity Speed (m/s)
 * @param trackSeg Track segment (curvature, camber)
 * @return Lateral acceleration demand in m/s², never negative
 */
double lateral_demand(double velocity, const TrackSegment& trackSeg) {
    const double k = std::abs(trackSeg.curvature);
    return std::abs(velocity * velocity * k * std::cos(trackSeg.camber) - constants::g * std::sin(trackSeg.camber));
}

/**
 * @brief Computes the maximum lateral acceleration the car can sustain.
 *
 * Friction force = μ · N, so the largest lateral acceleration is
 *
 *   a_lat_max = μ_lat · a_normal
 *
 * with μ_lat the tyres' lateral friction (TyreParams::base_grip) times the
 * surface's grip, and a_normal from normal_acceleration(): weight, banking
 * and downforce. Downforce adds grip without adding mass, which is why an
 * F1 car corners at 5-6 g where a road car manages about 1 g.
 *
 * Different surfaces scale the friction through the segment's grip:
 *   - Dry asphalt: grip ≈ 1.0
 *   - Wet track:   grip ≈ 0.7
 *   - Gravel:      grip ≈ 0.4
 *
 * @param state Current car state (velocity affects downforce)
 * @param vehicle Vehicle parameters (mass, aero, tyres)
 * @param trackSeg Track segment (grip, curvature, camber)
 * @return Maximum sustainable lateral acceleration in m/s²
 */
double compute_lateral_acc_limit(const CarState& state,
                                 const VehicleParams& vehicle,
                                 const TrackSegment& trackSeg) {
    const double mu = vehicle.tyre.base_grip * trackSeg.grip;
    return mu * normal_acceleration(state.v, vehicle, trackSeg);
}

/**
 * @brief Fraction of the longitudinal grip left after cornering.
 *
 * === The Friction Ellipse ===
 *
 * A tyre has one grip budget for cornering and for driving or braking. The
 * combinations it can hold fill an ellipse:
 *
 *   (a_long / a_long_max)² + (a_lat / a_lat_max)² ≤ 1
 *
 * so the share of the longitudinal grip left once the corner has taken its
 * share is
 *
 *   scale = √(1 − (a_lat / a_lat_max)²)
 *
 * On a straight all of it is left; at the cornering limit none is. The
 * ellipse's axes are the lateral and longitudinal friction
 * (TyreParams::base_grip and longitudinal_grip) on the same normal load.
 *
 * @param state Current car state (velocity for the demand and downforce)
 * @param vehicle Vehicle parameters
 * @param trackSeg Track segment (curvature determines the lateral demand)
 * @return Fraction of longitudinal grip available [0.0, 1.0]
 */
double compute_traction_scale(const CarState& state,
                              const VehicleParams& vehicle,
                              const TrackSegment& trackSeg) {
    const double a_lat_max = compute_lateral_acc_limit(state, vehicle, trackSeg);
    if (a_lat_max <= 0.0) {
        return 0.0;
    }

    const double usage = lateral_demand(state.v, trackSeg) / a_lat_max;
    if (usage >= 1.0) {
        return 0.0;  // at or past the cornering limit: nothing left
    }
    return std::sqrt(1.0 - usage * usage);
}

/**
 * @brief Largest force the tyres can transmit along the road now.
 *
 *   F_long_max = μ_long · m · a_normal · scale
 *
 * with μ_long the longitudinal friction (TyreParams::longitudinal_grip
 * times the surface's grip) and scale from the friction ellipse. Both the
 * drive force and the brake force are limited by it: the engine and the
 * brakes can ask for more than the tyres transmit.
 *
 * @param state Current car state
 * @param vehicle Vehicle parameters
 * @param trackSeg Track segment
 * @return Force in Newtons (>= 0)
 */
double longitudinal_grip_force(const CarState& state,
                               const VehicleParams& vehicle,
                               const TrackSegment& trackSeg) {
    return longitudinal_grip_force(state, vehicle, trackSeg, compute_traction_scale(state, vehicle, trackSeg));
}

double longitudinal_grip_force(const CarState& state,
                               const VehicleParams& vehicle,
                               const TrackSegment& trackSeg,
                               double traction_scale) {
    const double mu = vehicle.tyre.longitudinal_grip * trackSeg.grip;
    const double normal = std::max(0.0, normal_acceleration(state.v, vehicle, trackSeg));
    return mu * vehicle.mass * normal * traction_scale;
}

} // namespace physics
