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

TyreState average(const TyreState& a, const TyreState& b) {
    TyreState s;
    s.tread_temp = 0.5 * (a.tread_temp + b.tread_temp);
    s.carcass_temp = 0.5 * (a.carcass_temp + b.carcass_temp);
    s.wear = 0.5 * (a.wear + b.wear);
    s.pressure = 0.5 * (a.pressure + b.pressure);
    s.grip = 0.5 * (a.grip + b.grip);
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

    // Across each axle the corner moves load onto the outside tyre. The tyres push the car towards the
    // corner's inside (left for a positive curvature, less the banking's share), so the load goes right.
    const double towards_left = v * v * segment.curvature * std::cos(segment.camber) - constants::g * std::sin(segment.camber);
    const double signed_lateral = towards_left >= 0.0 ? lateral : -lateral;
    const auto right_share = [signed_lateral](const AxleTyres& axle, double load) {
        if (load <= 0.0) return 0.5;
        return std::clamp(0.5 + axle.lateral_transfer * signed_lateral / load, 0.0, 1.0);
    };
    front.right_share = right_share(p.front, front.load);
    rear.right_share = right_share(p.rear, rear.load);

    if (tyre_force >= 0.0) {
        front.force_long = p.front_driven ? tyre_force : 0.0;
        rear.force_long = p.front_driven ? 0.0 : tyre_force;
    } else {
        front.force_long = tyre_force * p.brake_front;
        rear.force_long = tyre_force * (1.0 - p.brake_front);
    }
}

double sliding_power(const AxleTyres& p, const AxleWork& work, double grip, double v) {
    if (work.load <= 0.0 || v <= 0.0) return 0.0;
    const double ux = std::abs(work.force_long) / std::max(work.mu_long * grip * work.load, 1e-9);
    const double uy = std::abs(work.force_lat) / std::max(work.mu_lat * grip * work.load, 1e-9);
    const double u = std::sqrt(ux * ux + uy * uy);
    if (u <= 0.0) return 0.0;
    // The force rises with the normalised slip s as 2s − s²: the grip used tells the slip.
    const double s = 1.0 - std::sqrt(1.0 - std::min(u, 1.0));
    const double slide_long = s * ux / u * p.peak_slip_ratio * v;
    const double slide_lat = std::tan(s * uy / u * p.peak_slip_angle) * v;
    return (std::abs(work.force_long) * slide_long + std::abs(work.force_lat) * slide_lat) * p.sliding_work;
}

void advance_tyre(const AxleTyres& p, const TyreConditionParams& condition, double load, double sliding, double v,
                  double dt, TyreState& state) {
    const TyreThermalParams& th = p.thermal;
    const TyrePressureParams& pr = p.pressure;

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

double load_factor(const AxleTyres& p, double load) {
    if (p.load_sensitivity == 0.0 || p.reference_load <= 0.0) return 1.0;
    return std::max(0.3, 1.0 + p.load_sensitivity * (load / p.reference_load - 1.0));
}

void advance(const AxleTyres& p, const TyreConditionParams& condition, const AxleWork& work, double v, double dt,
             TyreState& left, TyreState& right) {
    const double right_load = work.right_share * work.load;
    const double left_load = work.load - right_load;
    // The axle's grip is its tyres' friction, each on its load; each takes the forces, and the work, by its
    // share of it.
    const double left_force = left.grip * load_factor(p, left_load) * left_load;
    const double right_force = right.grip * load_factor(p, right_load) * right_load;
    const double grip = work.load > 0.0 ? (left_force + right_force) / work.load : 0.0;
    const double right_part = left_force + right_force > 0.0 ? right_force / (left_force + right_force) : work.right_share;
    const double sliding = sliding_power(p, work, grip, v);
    advance_tyre(p, condition, left_load, (1.0 - right_part) * sliding, v, dt, left);
    advance_tyre(p, condition, right_load, right_part * sliding, v, dt, right);
}

} // namespace physics::tyres
