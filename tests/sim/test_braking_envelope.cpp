#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sim/braking_envelope.hpp"
#include "physics/lateral.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"

/**
 * @brief A straight, a hairpin, a straight and a sweeper, closed into a loop.
 */
static Track create_test_track() {
    Track track;
    track.segments.push_back({400.0, 0.0, 1.0, 0.0});
    track.segments.push_back({94.2, 0.0333, 1.0, 0.0});   // 30 m hairpin, 180 degrees
    track.segments.push_back({300.0, 0.0, 1.0, 0.0});
    track.segments.push_back({314.2, 0.01, 1.0, 0.0});    // 100 m radius, 180 degrees
    track.total_length = 0.0;
    for (const auto& seg : track.segments) {
        track.total_length += seg.length;
    }
    return track;
}

static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};
    vehicle.mass = 1200.0;
    vehicle.wheel_radius = 0.32;
    vehicle.aero.drag_coefficient = 0.4;
    vehicle.aero.lift_coefficient = -1.0;
    vehicle.aero.frontal_area = 1.8;
    vehicle.drivetrain.engine.rpm = {2000, 5000, 8000};
    vehicle.drivetrain.engine.torque = {350, 450, 380};
    vehicle.drivetrain.gearbox.ratios = {3.2, 2.2, 1.6, 1.25, 1.0};
    vehicle.drivetrain.gearbox.final_drive = 3.7;
    vehicle.drivetrain.efficiency = 0.9;
    vehicle.max_drive_force = 12000.0;
    vehicle.max_brake_force = 25000.0;
    vehicle.tyre.base_grip = 1.3;
    vehicle.tyre.longitudinal_grip = 1.4;
    return vehicle;
}

TEST(BrakingEnvelope, NeverAboveTheCornerLimit) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    const BrakingEnvelope envelope = compute_braking_envelope(track, vehicle, 0.5);

    ASSERT_EQ(envelope.cap.size(), envelope.limit.size());
    for (size_t i = 0; i < envelope.limit.size(); ++i) {
        ASSERT_LE(envelope.limit[i], envelope.cap[i]) << "node " << i;
    }

    // The hairpin's own speed is the lap's slowest, and the envelope reaches it.
    const double hairpin = physics::lateral::max_speed(track.segments[1], vehicle);
    EXPECT_DOUBLE_EQ(*std::min_element(envelope.limit.begin(), envelope.limit.end()), hairpin);
    EXPECT_NEAR(envelope.speed_at(450.0), hairpin, 1e-9);
}

TEST(BrakingEnvelope, BrakingZonesAskNoMoreThanTheCarHas) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    const BrakingEnvelope envelope = compute_braking_envelope(track, vehicle, 0.5);

    // Deceleration between nodes: never more than full brakes (within grip) plus drag at the faster speed.
    const size_t n = envelope.limit.size();
    int braking_nodes = 0;
    for (size_t i = 0; i < n; ++i) {
        const double v = envelope.limit[i];
        const double v_next = envelope.limit[(i + 1) % n];
        if (v >= BrakingEnvelope::unlimited || v <= v_next) {
            continue;
        }
        CarState state{};
        state.v = v;
        const TrackSegment straight{1.0, 0.0, 1.0, 0.0};
        const physics::LongitudinalForces forces = physics::longitudinal_forces(state, vehicle, straight);
        const double most = (std::min(forces.brakes, forces.grip) + forces.drag) / vehicle.mass;
        const double asked = (v * v - v_next * v_next) / (2.0 * envelope.spacing);
        ASSERT_LE(asked, most * (1.0 + 1e-9)) << "node " << i;
        ++braking_nodes;
    }
    EXPECT_GT(braking_nodes, 100);

    // It starts braking for the hairpin well before it.
    EXPECT_LT(envelope.speed_at(380.0), envelope.speed_at(200.0));
}

TEST(BrakingEnvelope, WrapsRoundTheLap) {
    const Track track = create_test_track();
    const BrakingEnvelope envelope = compute_braking_envelope(track, create_test_vehicle(), 0.5);

    EXPECT_DOUBLE_EQ(envelope.length, track.total_length);
    EXPECT_NEAR(envelope.speed_at(track.total_length + 123.4), envelope.speed_at(123.4), 1e-9);
    EXPECT_NEAR(envelope.speed_at(-10.0), envelope.speed_at(track.total_length - 10.0), 1e-9);
}

TEST(BrakingEnvelope, AShortCornerBetweenNodesIsNotMissed) {
    Track track;
    track.segments.push_back({500.0, 0.0, 1.0, 0.0});
    track.segments.push_back({0.2, 0.1, 1.0, 0.0});   // a 10 m radius kink shorter than a node spacing
    track.segments.push_back({499.8, 0.0, 1.0, 0.0});
    track.total_length = 1000.0;
    const VehicleParams vehicle = create_test_vehicle();

    const BrakingEnvelope envelope = compute_braking_envelope(track, vehicle, 0.5);
    EXPECT_DOUBLE_EQ(*std::min_element(envelope.limit.begin(), envelope.limit.end()),
                     physics::lateral::max_speed(track.segments[1], vehicle));
}

TEST(BrakingEnvelope, RejectsBadInput) {
    const Track track = create_test_track();
    EXPECT_THROW(compute_braking_envelope(track, create_test_vehicle(), 0.0), std::invalid_argument);
    EXPECT_THROW(compute_braking_envelope(Track{}, create_test_vehicle(), 0.5), std::invalid_argument);
}
