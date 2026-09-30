#pragma once

#include "core/units.hpp"

struct SimConfig {
    seconds dt;
    seconds max_time;  // optional safety cap (out lap included)

    // Time a flying lap: drive an untimed out lap from rest first, and start
    // the clock as the car crosses the line at speed.
    bool flying_lap = false;

    // Node spacing of the braking envelope along the track (m).
    meters envelope_spacing = 0.5;
};
