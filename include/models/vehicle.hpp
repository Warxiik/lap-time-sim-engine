#pragma once

#include "models/aero.hpp"
#include "models/drivetrain.hpp"
#include "models/fuel.hpp"
#include "models/tyre.hpp"
#include "core/units.hpp"

struct VehicleParams {
    kilograms mass;  // the car without the fuel in `fuel` (all of it, when there is none)
    meters wheel_radius;

    newtons max_brake_force;
    newtons max_drive_force;

    Aero aero;
    Drivetrain drivetrain;
    TyreParams tyre;
    FuelParams fuel;                     // none by default
    TyreConditionParams tyre_condition;  // off by default
};
