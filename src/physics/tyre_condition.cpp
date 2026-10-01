#include "physics/tyre_condition.hpp"

#include <algorithm>
#include <cmath>

#include "core/constants.hpp"
#include "physics/aero.hpp"
#include "physics/tyre_model.hpp"

namespace physics::tyres {

namespace {

double smoothstep01(double x) {
    const double t = std::clamp(x, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

double temperature_grip(const TyreThermalParams& p, double tread_temp) {
    const double x = (tread_temp - p.optimal_temp) / p.temp_window;
    return std::max(p.min_temp_grip, 1.0 - p.grip_loss_at_window * x * x);
}

double wear_grip(const TyreWearParams& p, double wear) {
    const double w = std::clamp(wear, 0.0, 1.0);
    const double cliff = smoothstep01((w - p.cliff_start) / (1.0 - p.cliff_start));
    return 1.0 - p.grip_loss_linear * w - p.cliff_loss * cliff;
}

double pressure_grip(const TyrePressureParams& p, double pressure) {
    const double x = (pressure - p.optimal) / p.window;
    return std::max(p.min_grip, 1.0 - p.grip_loss_at_window * x * x);
}

double hot_pressure(const TyreThermalParams& p, double carcass_temp) {
    return p.cold_pressure * (carcass_temp + 273.15) / 293.15;
}

TyreState fresh(const AxleTyres& p) {
    TyreState s;
    s.tread_temp = p.thermal.initial_temp;
    s.carcass_temp = p.thermal.initial_temp;
    s.wear = 0.0;
    s.pressure = hot_pressure(p.thermal, s.carcass_temp);
    s.grip = temperature_grip(p.thermal, s.tread_temp) * wear_grip(p.wear, 0.0) * pressure_grip(p.pressure, s.pressure);
    return s;
}

void split(const TyreConditionParams& p, const VehicleParams& vehicle, double mass, double v, double tyre_force,
           const TrackSegment& segment, AxleWork& front, AxleWork& rear) {
    CarState at_speed{};
    at_speed.v = v;
    const double downforce = aero::downforce(vehicle, at_speed);
    const double k = std::abs(segment.curvature);
    const double weight = mass * (constants::g * std::cos(segment.camber) + v * v * k * std::sin(segment.camber));
    const double transfer = tyre_force * p.cg_height_over_wheelbase;  // driving moves load onto the rear axle
    front.load = std::max(0.0, weight * p.weight_front + downforce * p.aero_front - transfer);
    rear.load = std::max(0.0, weight * (1.0 - p.weight_front) + downforce * (1.0 - p.aero_front) + transfer);

    // A steady corner's yaw balance shares the lateral force by the static weight on each axle.
    const double lateral = mass * lateral_demand(v, segment);
    front.force_lat = lateral * p.weight_front;
    rear.force_lat = lateral * (1.0 - p.weight_front);

    if (tyre_force >= 0.0) {
        front.force_long = p.front_driven ? tyre_force : 0.0;
        rear.force_long = p.front_driven ? 0.0 : tyre_force;
    } else {
        front.force_long = tyre_force * p.brake_front;
        rear.force_long = tyre_force * (1.0 - p.brake_front);
    }
}

double sliding_power(const AxleTyres& p, const AxleWork& work, double v, double sliding_work) {
    if (work.load <= 0.0 || v <= 0.0) return 0.0;
    const double ux = std::abs(work.force_long) / std::max(work.mu_long * work.load, 1e-9);
    const double uy = std::abs(work.force_lat) / std::max(work.mu_lat * work.load, 1e-9);
    const double u = std::sqrt(ux * ux + uy * uy);
    if (u <= 0.0) return 0.0;
    // The force rises with the normalised slip s as 2s − s²: the grip used tells the slip.
    const double s = 1.0 - std::sqrt(1.0 - std::min(u, 1.0));
    const double slide_long = s * ux / u * p.peak_slip_ratio * v;
    const double slide_lat = std::tan(s * uy / u * p.peak_slip_angle) * v;
    return 0.5 * (std::abs(work.force_long) * slide_long + std::abs(work.force_lat) * slide_lat) * sliding_work;
}

void advance(const AxleTyres& p, const TyreConditionParams& condition, const AxleWork& work, double v, double dt,
             TyreState& state) {
    const TyreThermalParams& th = p.thermal;
    const TyrePressureParams& pr = p.pressure;
    const double load = 0.5 * work.load;  // one of the axle's two tyres
    const double sliding = sliding_power(p, work, v, condition.sliding_work);

    const double surface = state.tread_temp;
    const double carcass = state.carcass_temp;
    const double pressure = state.pressure;
    const double flex = std::pow(pr.optimal / std::max(pressure, 1.0), pr.flex_heat_exponent);
    const double into_surface = th.slip_heat_fraction * sliding;
    const double into_carcass = th.rolling_heat * load * v * flex;
    const double conduction = th.surface_carcass_conductance * (surface - carcass);
    const double surface_loss = (th.surface_cooling + th.surface_cooling_per_speed * v) * (surface - condition.ambient_temp) +
                                th.road_conductance * (surface - condition.track_temp);
    const double carcass_loss = (th.carcass_cooling + th.carcass_cooling_per_speed * v) * (carcass - condition.ambient_temp);
    state.tread_temp = surface + dt * (into_surface - conduction - surface_loss) / th.surface_heat_capacity;
    state.carcass_temp = carcass + dt * (into_carcass + conduction - carcass_loss) / th.carcass_heat_capacity;
    state.pressure = hot_pressure(th, state.carcass_temp);

    const TyreWearParams& we = p.wear;
    const double overheat = std::min(we.overheat_max, 1.0 + we.overheat_factor * std::max(0.0, surface - we.overheat_temp));
    const double off_optimum = std::min(3.0, std::abs(pressure - pr.optimal) / pr.window);
    state.wear = std::min(1.0, state.wear + dt * we.rate * sliding * overheat * (1.0 + pr.wear_at_window * off_optimum * off_optimum));

    state.grip = temperature_grip(th, state.tread_temp) * wear_grip(we, state.wear) * pressure_grip(pr, state.pressure);
}

} // namespace physics::tyres
