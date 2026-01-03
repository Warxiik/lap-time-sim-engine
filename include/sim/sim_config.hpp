#pragma once

#include "core/units.hpp"

struct SimConfig {
    seconds dt;
    seconds max_time;  // optional safety cap
};