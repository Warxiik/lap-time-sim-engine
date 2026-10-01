#include "sim/segment_index.hpp"

#include <algorithm>
#include <cmath>

SegmentIndex::SegmentIndex(const Track& track) : total_length_(track.total_length) {
    ends_.reserve(track.segments.size());
    double accumulated = 0.0;
    for (const auto& segment : track.segments) {
        accumulated += segment.length;
        ends_.push_back(accumulated);
    }
}

std::size_t SegmentIndex::find(double position) const {
    if (ends_.empty()) return 0;
    const double wrapped = std::fmod(position, total_length_);
    const auto it = std::upper_bound(ends_.begin(), ends_.end(), wrapped);
    if (it == ends_.end()) return ends_.size() - 1;
    return static_cast<std::size_t>(it - ends_.begin());
}
