#pragma once

#include "models/aero.hpp"
#include "models/drivetrain.hpp"
#include "core/units.hpp"

struct VehicleParams {
    kilograms mass;
    meters wheel_radius;

    newtons max_brake_force;
    newtons max_drive_force;

    Aero aero;
    Drivetrain drivetrain;
};