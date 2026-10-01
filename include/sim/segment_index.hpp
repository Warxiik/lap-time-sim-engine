#pragma once

#include <cstddef>
#include <vector>

#include "models/track.hpp"

/**
 * @file segment_index.hpp
 * @brief Which segment of a track a distance falls in, found by bisection.
 *
 * A racing line cut into pieces has a thousand segments or more, and every
 * step of a lap asks which one the car is in. The index holds where each
 * segment ends (summed in order, as a walk along the segments would) and
 * bisects them, so a step costs a few comparisons instead of a walk along
 * the whole lap.
 */
class SegmentIndex {
public:
    explicit SegmentIndex(const Track& track);

    /// Index of the segment at `position` m along the lap (wrapping round it): the first segment that
    /// ends beyond it, or the last one.
    [[nodiscard]] std::size_t find(double position) const;

private:
    std::vector<double> ends_;
    double total_length_ = 0.0;
};
