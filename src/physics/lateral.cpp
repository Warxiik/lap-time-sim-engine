#include "physics/lateral.hpp"
#include "physics/aero.hpp"
#include "physics/tyre_model.hpp"
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
 * This force comes from tire friction. If you go too fast, the required
 * centripetal force exceeds available grip, and you slide off the track.
 *
 * Centripetal acceleration: a_lat = v² / r = v² * κ
 *   Where κ (kappa) is curvature = 1/radius
 *
 * Maximum lateral acceleration from tires: a_lat_max = μ * g_effective
 *   Where g_effective includes downforce contribution
 *
 * Setting a_lat = a_lat_max and solving for v:
 *   v_max = sqrt(a_lat_max / κ)
 *
 * === Why Downforce Matters ===
 *
 * Downforce increases the normal force on tires without adding mass.
 * More normal force = more friction force available.
 * This is why F1 cars can corner at 5-6g while road cars max out at ~1g.
 *
 * However, there's a catch: downforce is proportional to v².
 * So we need to solve iteratively or use the pre-computed lateral
 * acceleration limit that accounts for this.
 *
 * === Curvature Sign Convention ===
 *
 * Curvature κ = 1/radius
 *   - κ = 0: straight line (infinite radius, no lateral limit)
 *   - κ > 0: left turn
 *   - κ < 0: right turn
 *   - |κ| larger = tighter turn
 *
 * We use absolute value since direction doesn't affect speed limit.
 *
 * @param segment Track segment with curvature and grip info
 * @param vehicle Vehicle parameters (mass, aero)
 * @param state Current car state (used for downforce calculation)
 * @return Maximum safe cornering speed in m/s
 */
double max_speed(const TrackSegment& segment,
                 const VehicleParams& vehicle,
                 const CarState& state) {

    const double curvature = std::abs(segment.curvature);

    // Straight section: no lateral acceleration limit
    // Return a very large number (effectively unlimited by cornering)
    // Actual speed will be limited by engine power and drag
    if (curvature < 1e-9) {
        return std::numeric_limits<double>::max();
    }

    // Get the lateral acceleration limit at current state
    // This already accounts for downforce at the current velocity
    const double a_lat_max = physics::compute_lateral_acc_limit(state, vehicle, segment);

    // Guard against zero or negative grip (shouldn't happen, but be safe)
    if (a_lat_max <= 0.0) {
        return 0.0;
    }

    // v_max = sqrt(a_lat_max / curvature)
    // Derivation: a_lat = v² * curvature → v = sqrt(a_lat / curvature)
    return std::sqrt(a_lat_max / curvature);
}

} // namespace physics::lateral
