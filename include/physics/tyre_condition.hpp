#pragma once

#include "models/car_state.hpp"
#include "models/track.hpp"
#include "models/tyre.hpp"
#include "models/vehicle.hpp"

/**
 * @file tyre_condition.hpp
 * @brief The tyres' temperature, wear and pressure over a stint (TyreConditionParams).
 */
namespace physics::tyres {

    /// Grip multiplier from the tread temperature: a parabola around the optimum, floored.
    double temperature_grip(const TyreThermalParams& p, double tread_temp);
    /// Grip multiplier from wear (0 new .. 1 worn): linear loss, then a cliff.
    double wear_grip(const TyreWearParams& p, double wear);
    /// Grip multiplier from the hot pressure: a parabola around the optimum, floored.
    double pressure_grip(const TyrePressureParams& p, double pressure);
    /// Hot pressure (kPa): the cold pressure (at 20 °C) scaled with the carcass temperature (ideal gas).
    double hot_pressure(const TyreThermalParams& p, double carcass_temp);

    /// Fresh tyres at the stint's start: both nodes at initial_temp, no wear.
    TyreState fresh(const AxleTyres& p);

    /// What a step does to one axle's tyres, for the parts that differ by axle.
    struct AxleWork {
        double load = 0.0;        // N on the axle
        double force_long = 0.0;  // N along the road (+ driving)
        double force_lat = 0.0;   // N across it
        double mu_long = 1.0;     // the axle's friction now (TyreParams × its condition × the surface)
        double mu_lat = 1.0;
    };

    /// The forces and load on each axle when the car of `mass` kg carries `tyre_force` N along the road
    /// (+ driving) at speed `v` in `segment`.
    void split(const TyreConditionParams& p, const VehicleParams& vehicle, double mass, double v, double tyre_force,
               const TrackSegment& segment, AxleWork& front, AxleWork& rear);

    /// Sliding power of one of the axle's two tyres (W): its share of the forces times the sliding speed the
    /// grip it uses implies, times `sliding_work`.
    double sliding_power(const AxleTyres& p, const AxleWork& work, double v, double sliding_work);

    /// Advances one axle's tyres by `dt` at speed `v`.
    void advance(const AxleTyres& p, const TyreConditionParams& condition, const AxleWork& work, double v, double dt,
                 TyreState& state);

} // namespace physics::tyres
