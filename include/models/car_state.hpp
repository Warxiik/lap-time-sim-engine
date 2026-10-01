#pragma once

#include "core/units.hpp"
#include "models/tyre.hpp"

struct CarState {
    meters s; // Position along the track

    // Kinematics
    meters_ps v; // speed
    meters_ps2 a; // acceleration

    // Powertrain state
    int gear;
    double engine_rpm;

    // Consumables: what is left of the fuel, and each axle's tyres
    double fuel = 0.0;  // kg
    TyreState front_tyres;
    TyreState rear_tyres;
};