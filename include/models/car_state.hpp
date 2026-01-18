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

    // Tire state
    TyreState tyres;
};