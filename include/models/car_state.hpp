#pragma once

#include <array>
#include <cstddef>

#include "core/units.hpp"
#include "models/tyre.hpp"

/// Where each tyre is in CarState::tyres.
struct TyreIndex {
    static constexpr std::size_t front_left = 0;
    static constexpr std::size_t front_right = 1;
    static constexpr std::size_t rear_left = 2;
    static constexpr std::size_t rear_right = 3;
};

struct CarState {
    meters s; // Position along the track

    // Kinematics
    meters_ps v; // speed
    meters_ps2 a; // acceleration

    // Powertrain state
    int gear;
    double engine_rpm;

    // Consumables: what is left of the fuel, and the tyres (TyreIndex)
    double fuel = 0.0;  // kg
    std::array<TyreState, 4> tyres{};
};