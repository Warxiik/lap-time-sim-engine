#include <gtest/gtest.h>

#include <cmath>

#include "sim/segment_index.hpp"
#include "models/track.hpp"

/// The segment a walk along the lap finds: the first one that ends beyond the position, or the last.
static size_t walk(const Track& track, double position) {
    const double wrapped = std::fmod(position, track.total_length);
    double accumulated = 0.0;
    for (size_t i = 0; i < track.segments.size(); ++i) {
        accumulated += track.segments[i].length;
        if (wrapped < accumulated) {
            return i;
        }
    }
    return track.segments.size() - 1;
}

static Track create_test_track() {
    Track track;
    for (int k = 0; k < 1200; ++k) {
        track.segments.push_back({1.5 + 0.7 * std::sin(0.37 * k), 0.0, 1.0, 0.0});
    }
    track.total_length = 0.0;
    for (const auto& seg : track.segments) {
        track.total_length += seg.length;
    }
    return track;
}

/**
 * @test The index finds the segment a walk along the lap does, at every boundary and past the line.
 */
TEST(SegmentIndex, FindsWhatAWalkFinds) {
    const Track track = create_test_track();
    const SegmentIndex index(track);

    double end = 0.0;
    for (size_t i = 0; i < track.segments.size(); ++i) {
        const double start = end;
        end += track.segments[i].length;
        for (double s : {start, std::nextafter(start, end), 0.5 * (start + end), std::nextafter(end, start)}) {
            ASSERT_EQ(index.find(s), walk(track, s)) << "at " << s;
            ASSERT_EQ(index.find(s + track.total_length), walk(track, s + track.total_length)) << "a lap on, at " << s;
        }
        ASSERT_EQ(index.find(start), i);
    }
    EXPECT_EQ(index.find(0.0), 0u);
    EXPECT_EQ(index.find(track.total_length), 0u);  // the line is the next lap's start
}

/**
 * @test One segment is always that segment.
 */
TEST(SegmentIndex, OneSegment) {
    Track track;
    track.segments.push_back({100.0, 0.01, 1.0, 0.0});
    track.total_length = 100.0;
    const SegmentIndex index(track);
    EXPECT_EQ(index.find(0.0), 0u);
    EXPECT_EQ(index.find(99.999), 0u);
    EXPECT_EQ(index.find(250.0), 0u);
}
