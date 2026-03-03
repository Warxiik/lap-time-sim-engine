#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>

#include "io/track_loader.hpp"

/**
 * @brief Helper to write a temporary CSV file for testing.
 */
static std::string write_temp_csv(const std::string& content) {
    std::string path = "test_track_tmp.csv";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

static void remove_temp(const std::string& path) {
    std::remove(path.c_str());
}

// =============================================================================
// Track Loader Tests
// =============================================================================

TEST(TrackLoader, LoadsValidTrack) {
    auto path = write_temp_csv(
        "length,curvature,grip,camber\n"
        "100.0,0.0,1.0,0.0\n"
        "50.0,0.02,1.0,0.01\n"
        "75.0,-0.01,0.95,0.0\n"
    );

    Track track = load_track(path);
    remove_temp(path);

    ASSERT_EQ(track.segments.size(), 3u);
    EXPECT_DOUBLE_EQ(track.total_length, 225.0);

    EXPECT_DOUBLE_EQ(track.segments[0].length, 100.0);
    EXPECT_DOUBLE_EQ(track.segments[0].curvature, 0.0);

    EXPECT_DOUBLE_EQ(track.segments[1].length, 50.0);
    EXPECT_DOUBLE_EQ(track.segments[1].curvature, 0.02);

    EXPECT_DOUBLE_EQ(track.segments[2].grip, 0.95);
}

TEST(TrackLoader, SkipsCommentLines) {
    auto path = write_temp_csv(
        "length,curvature,grip,camber\n"
        "# This is a comment\n"
        "100.0,0.0,1.0,0.0\n"
    );

    Track track = load_track(path);
    remove_temp(path);

    ASSERT_EQ(track.segments.size(), 1u);
    EXPECT_DOUBLE_EQ(track.total_length, 100.0);
}

TEST(TrackLoader, ThrowsOnMissingFile) {
    EXPECT_THROW(load_track("nonexistent_file.csv"), std::runtime_error);
}

TEST(TrackLoader, ThrowsOnEmptyFile) {
    auto path = write_temp_csv("");
    EXPECT_THROW(load_track(path), std::runtime_error);
    remove_temp(path);
}

TEST(TrackLoader, ThrowsOnHeaderOnly) {
    auto path = write_temp_csv("length,curvature,grip,camber\n");
    EXPECT_THROW(load_track(path), std::runtime_error);
    remove_temp(path);
}

TEST(TrackLoader, ThrowsOnNegativeLength) {
    auto path = write_temp_csv(
        "length,curvature,grip,camber\n"
        "-10.0,0.0,1.0,0.0\n"
    );
    EXPECT_THROW(load_track(path), std::runtime_error);
    remove_temp(path);
}

TEST(TrackLoader, ThrowsOnZeroGrip) {
    auto path = write_temp_csv(
        "length,curvature,grip,camber\n"
        "100.0,0.0,0.0,0.0\n"
    );
    EXPECT_THROW(load_track(path), std::runtime_error);
    remove_temp(path);
}

TEST(TrackLoader, ThrowsOnMalformedLine) {
    auto path = write_temp_csv(
        "length,curvature,grip,camber\n"
        "not,valid,data\n"
    );
    EXPECT_THROW(load_track(path), std::runtime_error);
    remove_temp(path);
}
