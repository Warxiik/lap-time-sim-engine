#pragma once

#include <vector>
#include "core/units.hpp"

struct TrackSegment {
    meters length;
    double curvature;
    double grip;
    double camber;
};

struct Track {
    std::vector<TrackSegment> segments;
    meters total_length;
};