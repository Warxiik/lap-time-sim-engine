#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "sim/simulator.hpp"
#include "physics/lateral.hpp"
#include "physics/tyre_model.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "core/constants.hpp"

/**
 * @brief A straight, a hairpin, a straight, a sweeper and a chicane, closed into a loop.
 */
static Track create_test_track() {
    Track track;
    track.segments.push_back({400.0, 0.0, 1.0, 0.0});
    track.segments.push_back({94.2, 0.0333, 1.0, 0.0});   // 30 m hairpin
    track.segments.push_back({300.0, 0.0, 1.0, 0.0});
    track.segments.push_back({314.2, 0.01, 1.0, 0.05});   // 100 m radius, slightly banked
    track.segments.push_back({60.0, 0.0, 1.0, 0.0});
    track.segments.push_back({20.0, -0.04, 1.0, 0.0});    // chicane
    track.segments.push_back({20.0, 0.04, 1.0, 0.0});
    track.segments.push_back({100.0, 0.0, 1.0, 0.0});
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

constexpr double kPi = 3.14159265358979323846;

static SimConfig create_config(bool flying) {
    SimConfig config{};
    config.dt = 0.001;
    config.max_time = 600.0;
    config.flying_lap = flying;
    return config;
}

static double lap_time(const Track& track, const VehicleParams& vehicle, bool flying) {
    Simulator sim(track, vehicle, create_config(flying));
    sim.run();
    EXPECT_TRUE(sim.completed());
    return sim.lap_time();
}

static const TrackSegment& segment_at(const Track& track, double s) {
    double end = 0.0;
    for (const auto& seg : track.segments) {
        end += seg.length;
        if (s < end) {
            return seg;
        }
    }
    return track.segments.back();
}

/**
 * @test Nothing clamps the speed, and the car still keeps within its grip in every corner.
 */
TEST(Lap, KeepsWithinTheCornerLimitWithoutAClamp) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    Simulator sim(track, vehicle, create_config(true));
    sim.run();
    ASSERT_TRUE(sim.completed());

    const Telemetry& t = sim.telemetry();
    double worst_usage = 0.0;
    for (size_t k = 0; k < t.size(); ++k) {
        const TrackSegment& seg = segment_at(track, std::fmod(t.distance[k], track.total_length));
        CarState state{};
        state.v = t.velocity[k];
        const double usage = physics::lateral_demand(state.v, seg) / physics::compute_lateral_acc_limit(state, vehicle, seg);
        worst_usage = std::max(worst_usage, usage);
    }
    EXPECT_LT(worst_usage, 1.005);
    EXPECT_GT(worst_usage, 0.98);  // and it does use the grip
}

/**
 * @test Speed only changes as the forces allow: no step loses more than full braking and drag.
 */
TEST(Lap, NeverLosesSpeedFasterThanItCanBrake) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    const SimConfig config = create_config(true);
    Simulator sim(track, vehicle, config);
    sim.run();

    const Telemetry& t = sim.telemetry();
    for (size_t k = 1; k < t.size(); ++k) {
        CarState state{};
        state.v = t.velocity[k - 1];
        const TrackSegment straight{1.0, 0.0, 1.0, 0.0};
        const physics::LongitudinalForces forces = physics::longitudinal_forces(state, vehicle, straight);
        const double most = (std::min(forces.brakes, forces.grip) + forces.drag) / vehicle.mass * config.dt;
        ASSERT_LE(t.velocity[k - 1] - t.velocity[k], most * (1.0 + 1e-9)) << "frame " << k;
    }
}

/**
 * @test A heavier car is slower, standing or flying.
 */
TEST(Lap, HeavierIsSlower) {
    const Track track = create_test_track();
    const VehicleParams light = create_test_vehicle();
    VehicleParams heavy = light;
    heavy.mass += 60.0;  // a tank of fuel

    EXPECT_GT(lap_time(track, heavy, true), lap_time(track, light, true));
    EXPECT_GT(lap_time(track, heavy, false), lap_time(track, light, false));
}

/**
 * @test More tyre grip is faster.
 */
TEST(Lap, MoreGripIsFaster) {
    const Track track = create_test_track();
    const VehicleParams base = create_test_vehicle();
    VehicleParams grippier = base;
    grippier.tyre.base_grip *= 1.1;
    grippier.tyre.longitudinal_grip *= 1.1;
    EXPECT_LT(lap_time(track, grippier, true), lap_time(track, base, true));
}

/**
 * @test A flying lap starts at speed, is faster than a standing one, and ends as fast as it began.
 */
TEST(Lap, FlyingLapStartsAtSpeedAndCloses) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    Simulator flying(track, vehicle, create_config(true));
    flying.run();
    ASSERT_TRUE(flying.completed());

    const Telemetry& t = flying.telemetry();
    EXPECT_GT(t.velocity.front(), 20.0);
    EXPECT_NEAR(t.velocity.back(), t.velocity.front(), 0.2);
    EXPECT_LT(t.distance.front(), 0.1);
    EXPECT_LT(flying.lap_time(), lap_time(track, vehicle, false) - 1.0);

    // Telemetry time is the time of each state, within the timed lap.
    EXPECT_GT(t.time.front(), 0.0);
    EXPECT_LE(t.time.front(), 0.001 + 1e-12);
    EXPECT_NEAR(t.time.back(), flying.lap_time(), 0.001);
}

/**
 * @test Round a circle, the lap is the circle's length at its corner speed, less what drag costs.
 *
 * At the limit the ellipse leaves no grip to push against drag, so the car
 * settles just under the corner speed, where the grip left just matches it.
 */
TEST(Lap, CircleRunsAtItsCornerSpeed) {
    const double radius = 80.0;
    Track circle;
    circle.segments.push_back({2.0 * kPi * radius, 1.0 / radius, 1.0, 0.0});
    circle.total_length = circle.segments[0].length;
    const VehicleParams vehicle = create_test_vehicle();

    const double v_corner = physics::lateral::max_speed(circle.segments[0], vehicle);
    const double ideal = circle.total_length / v_corner;
    const double lap = lap_time(circle, vehicle, true);
    EXPECT_GT(lap, ideal);
    EXPECT_LT(lap, 1.01 * ideal);
}

/**
 * @test The lap time barely depends on the step: it is interpolated at the line.
 */
TEST(Lap, LapTimeHardlyDependsOnTheStep) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    SimConfig fine = create_config(true);
    fine.dt = 0.0005;
    Simulator a(track, vehicle, create_config(true));
    Simulator b(track, vehicle, fine);
    a.run();
    b.run();
    EXPECT_NEAR(a.lap_time(), b.lap_time(), 0.001 * b.lap_time());
}

/**
 * @test A corner that tightens over many short segments (as a racing line cut into pieces) is braked into
 * without ever passing its limit. Braking and cornering share the grip there, and past the limit none is
 * left to brake with: a car that fell behind the braking envelope at a segment boundary ran on through the
 * whole corner.
 */
TEST(Lap, BrakesIntoATighteningCorner) {
    Track track;
    track.segments.push_back({500.0, 0.0, 1.0, 0.0});
    for (int k = 1; k <= 8; ++k) {
        track.segments.push_back({10.0, -0.005 * k, 1.0, 0.0});  // radius 200 m down to 25 m
    }
    track.segments.push_back({60.0, -0.04, 1.0, 0.0});
    for (int k = 8; k >= 1; --k) {
        track.segments.push_back({10.0, -0.005 * k, 1.0, 0.0});
    }
    track.segments.push_back({400.0, 0.0, 1.0, 0.0});
    track.segments.push_back({2.0 * kPi * 60.0, 1.0 / 60.0, 1.0, 0.0});  // a loop of 60 m radius back to the start
    track.total_length = 0.0;
    for (const auto& seg : track.segments) {
        track.total_length += seg.length;
    }
    const VehicleParams vehicle = create_test_vehicle();
    Simulator sim(track, vehicle, create_config(true));
    sim.run();
    ASSERT_TRUE(sim.completed());

    const Telemetry& t = sim.telemetry();
    double worst_usage = 0.0;
    for (size_t k = 0; k < t.size(); ++k) {
        const TrackSegment& seg = segment_at(track, std::fmod(t.distance[k], track.total_length));
        CarState state{};
        state.v = t.velocity[k];
        worst_usage = std::max(worst_usage,
                               physics::lateral_demand(state.v, seg) / physics::compute_lateral_acc_limit(state, vehicle, seg));
    }
    EXPECT_LT(worst_usage, 1.005);
}

/**
 * @test The lap time moves smoothly with the car's mass: no step from one mass to the next jumps.
 *
 * A gear chosen by rpm thresholds remembers the gear it was in: leaving a corner just above the
 * downshift rpm, the car stayed a gear too high all the way down the next straight, and a few kilograms
 * more or less flipped which gear it left in. Every gram then cost the same, except at the flips.
 */
TEST(Lap, LapTimeIsSmoothInMass) {
    const Track track = create_test_track();
    VehicleParams vehicle = create_test_vehicle();
    std::vector<double> laps;
    for (int k = 0; k <= 60; ++k) {
        vehicle.mass = 1150.0 + 2.0 * k;
        laps.push_back(lap_time(track, vehicle, true));
    }
    const double per_kg = (laps.back() - laps.front()) / 120.0;
    ASSERT_GT(per_kg, 0.0);
    for (size_t k = 1; k < laps.size(); ++k) {
        const double step = laps[k] - laps[k - 1];
        EXPECT_GT(step, 0.0) << "at " << 1150.0 + 2.0 * k << " kg";
        EXPECT_LT(step, 3.0 * 2.0 * per_kg) << "at " << 1150.0 + 2.0 * k << " kg";
    }
}
