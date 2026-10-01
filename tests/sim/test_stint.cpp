#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "sim/simulator.hpp"
#include "physics/tyre_condition.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"

/**
 * @brief A straight, a hairpin, a straight, a sweeper and a chicane, closed into a loop (as test_lap's).
 */
static Track create_test_track() {
    Track track;
    track.segments.push_back({400.0, 0.0, 1.0, 0.0});
    track.segments.push_back({94.2, 0.0333, 1.0, 0.0});
    track.segments.push_back({300.0, 0.0, 1.0, 0.0});
    track.segments.push_back({314.2, 0.01, 1.0, 0.05});
    track.segments.push_back({60.0, 0.0, 1.0, 0.0});
    track.segments.push_back({20.0, -0.04, 1.0, 0.0});
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

static VehicleParams with_consumables(VehicleParams vehicle) {
    vehicle.fuel.mass = 60.0;
    vehicle.fuel.bsfc = 300.0;
    vehicle.fuel.idle_flow = 1e-4;
    TyreConditionParams& c = vehicle.tyre_condition;
    c.enabled = true;
    c.weight_front = 0.52;
    c.aero_front = 0.45;
    c.brake_front = 0.65;
    c.cg_height_over_wheelbase = 0.18;
    return vehicle;
}

static SimConfig stint(int laps) {
    SimConfig config{};
    config.dt = 0.001;
    config.max_time = 3600.0;
    config.flying_lap = true;
    config.laps = laps;
    return config;
}

/**
 * @test A stint of a car that burns nothing and never wears is its first lap again and again, and one lap
 * of it is the plain lap.
 */
TEST(Stint, WithoutConsumablesEveryLapIsTheSame) {
    const Track track = create_test_track();
    const VehicleParams vehicle = create_test_vehicle();
    Simulator one(track, vehicle, stint(1));
    one.run();
    Simulator three(track, vehicle, stint(3));
    three.run();
    ASSERT_TRUE(three.completed());
    ASSERT_EQ(three.laps().size(), 3u);
    EXPECT_EQ(three.laps()[0].time, one.lap_time());
    EXPECT_EQ(three.lap_time(), one.lap_time());
    for (const LapRecord& lap : three.laps()) EXPECT_NEAR(lap.time, one.lap_time(), 0.002);

    // The telemetry runs on from lap to lap, by distance and time.
    const Telemetry& t = three.telemetry();
    for (size_t k = 1; k < t.size(); ++k) {
        ASSERT_GE(t.distance[k], t.distance[k - 1]);
        ASSERT_GT(t.time[k], t.time[k - 1]);
    }
    EXPECT_GT(t.distance.back(), 3.0 * track.total_length);
    EXPECT_NEAR(t.time.back(), three.laps()[0].time + three.laps()[1].time + three.laps()[2].time, 0.001);

    // Not enough time for all three laps: not completed, but the laps that were driven are there.
    SimConfig short_time = stint(3);
    short_time.max_time = 3.5 * one.lap_time();  // the out lap from rest and two laps
    Simulator cut(track, vehicle, short_time);
    cut.run();
    EXPECT_FALSE(cut.completed());
    EXPECT_EQ(cut.laps().size(), 2u);
    EXPECT_EQ(cut.lap_time(), one.lap_time());
}

/**
 * @test The engine burns fuel with its work: every lap a similar amount, and the lighter car gets faster.
 */
TEST(Stint, FuelBurnsAndTheCarGetsFaster) {
    const Track track = create_test_track();
    VehicleParams vehicle = create_test_vehicle();
    vehicle.fuel.mass = 60.0;
    vehicle.fuel.bsfc = 300.0;
    vehicle.mass -= 60.0;  // the same car as above, but with its fuel separate
    Simulator sim(track, vehicle, stint(6));
    sim.run();
    ASSERT_TRUE(sim.completed());
    const auto& laps = sim.laps();
    const double first = laps[0].fuel_at_start - laps[0].fuel_at_end;
    EXPECT_GT(first, 0.2);
    EXPECT_LT(first, 2.0);
    for (size_t k = 1; k < laps.size(); ++k) {
        EXPECT_DOUBLE_EQ(laps[k].fuel_at_start, laps[k - 1].fuel_at_end);
        EXPECT_NEAR(laps[k].fuel_at_start - laps[k].fuel_at_end, first, 0.05 * first);
        EXPECT_LT(laps[k].time, laps[k - 1].time);
    }
    EXPECT_NEAR(sim.vehicle_now().mass, vehicle.mass + laps.back().fuel_at_end, 1e-9);

    // No consumption model: the fuel is carried as ballast, lap after lap.
    VehicleParams ballast = vehicle;
    ballast.fuel.bsfc = 0.0;
    Simulator heavy(track, ballast, stint(2));
    heavy.run();
    EXPECT_DOUBLE_EQ(heavy.laps()[1].fuel_at_end, 60.0);
    EXPECT_GT(heavy.laps()[1].time, laps[1].time);
}

/**
 * @test The tyres come up to temperature over the first laps, settle, wear lap after lap, and once the
 * tread is worn through to its cliff the laps get slower faster. (This car works its driven rear tyres
 * hard: they wear about a tenth of their tread a lap.)
 */
TEST(Stint, TyresWarmUpWearAndFallOffTheCliff) {
    const Track track = create_test_track();
    VehicleParams vehicle = with_consumables(create_test_vehicle());
    vehicle.fuel = FuelParams{};  // tyres alone
    Simulator sim(track, vehicle, stint(8));
    sim.run();
    ASSERT_TRUE(sim.completed());
    const auto& laps = sim.laps();

    // Warm: the tread is far above where the warmers left it, and close from one lap to the next.
    const double initial = vehicle.tyre_condition.rear.thermal.initial_temp;
    EXPECT_GT(laps[1].rear.tread_temp, initial + 15.0);
    EXPECT_NEAR(laps[3].rear.tread_temp, laps[2].rear.tread_temp, 5.0);
    // Worn: lap after lap, past the cliff by the end.
    for (size_t k = 1; k < laps.size(); ++k) EXPECT_GT(laps[k].rear.wear, laps[k - 1].rear.wear);
    EXPECT_GT(std::max(laps.back().front.wear, laps.back().rear.wear), vehicle.tyre_condition.rear.wear.cliff_start);
    // And slower for it, more so once the cliff comes.
    ASSERT_LT(laps.back().rear.wear, 1.0);
    EXPECT_GT(laps.back().time, laps[2].time + 0.3);
    const double early = laps[3].time - laps[2].time;
    const double late = laps[7].time - laps[6].time;
    EXPECT_GT(late, early);
    // Pressure follows the carcass.
    EXPECT_DOUBLE_EQ(laps.back().rear.pressure, physics::tyres::hot_pressure(vehicle.tyre_condition.rear.thermal, laps.back().rear.carcass_temp));
}

/**
 * @test A car whose grip changes as it drives has its braking planned again for the tyres it has: worn, it
 * takes the hairpin slower than on new tyres. The stint, fuel and tyres included, is deterministic.
 */
TEST(Stint, PlansItsBrakingForTheTyresItHas) {
    const Track track = create_test_track();
    const VehicleParams vehicle = with_consumables(create_test_vehicle());
    Simulator a(track, vehicle, stint(8));
    a.run();
    Simulator b(track, vehicle, stint(8));
    b.run();
    ASSERT_TRUE(a.completed());
    ASSERT_EQ(a.laps().size(), b.laps().size());
    for (size_t k = 0; k < a.laps().size(); ++k) EXPECT_EQ(a.laps()[k].time, b.laps()[k].time);
    EXPECT_EQ(a.telemetry().velocity, b.telemetry().velocity);

    // The worn car is slower through the hairpin: it does not carry the speed of new tyres.
    const Telemetry& t = a.telemetry();
    const double length = track.total_length;
    double first = 1e9;
    double last = 1e9;
    for (size_t k = 0; k < t.size(); ++k) {
        const double s = std::fmod(t.distance[k], length);
        if (s < 400.0 || s > 494.2) continue;
        if (t.distance[k] < 2.0 * length && t.distance[k] > length) first = std::min(first, t.velocity[k]);
        if (t.distance[k] > 7.0 * length) last = std::min(last, t.velocity[k]);
    }
    EXPECT_LT(last, first);
}
