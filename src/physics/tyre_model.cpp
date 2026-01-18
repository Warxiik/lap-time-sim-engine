#include "physics/tyre_model.hpp"
#include "core/constants.hpp"
#include <algorithm>

namespace physics {
    double compute_lateral_acc_limit(const CarState &state, const VehicleParams &vehicle, const TrackSegment &trackSeg) {
        const auto& aero = vehicle.aero;

        const double downforce = 0.5 * constants::air_density * aero.lift_coefficient * aero.frontal_area * state.v * state.v;
        const double normal_force = vehicle.mass * constants::g + downforce;

        const double mu = trackSeg.grip;

        return mu * normal_force / vehicle.mass;
    }

    double compute_traction_scale(const CarState &state, const VehicleParams &vehicle, const TrackSegment &trackSeg) {
        const double a_lat = state.v * state.v * trackSeg.curvature;
        const double a_lat_max = compute_lateral_acc_limit(state, vehicle, trackSeg);

        if (a_lat_max <= 0.0) return 1.0;

        const double scale = 1.0 - (a_lat / a_lat_max);
        return std::clamp(scale, 0.0, 1.0);
    }
} // namespace physics