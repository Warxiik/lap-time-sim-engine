#include <gtest/gtest.h>

#include <cmath>

#include "physics/tyre_condition.hpp"
#include "core/constants.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"

using namespace physics::tyres;

static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};
    vehicle.mass = 1000.0;
    vehicle.wheel_radius = 0.3;
    vehicle.aero.drag_coefficient = 0.35;
    vehicle.aero.lift_coefficient = -1.0;
    vehicle.aero.frontal_area = 1.5;
    vehicle.tyre.base_grip = 1.2;
    vehicle.tyre.longitudinal_grip = 1.25;
    TyreConditionParams& c = vehicle.tyre_condition;
    c.enabled = true;
    c.weight_front = 0.55;
    c.aero_front = 0.45;
    c.brake_front = 0.65;
    c.front_driven = false;
    c.cg_height_over_wheelbase = 0.2;
    return vehicle;
}

TEST(TyreCondition, GripCurvesPeakAtTheirOptimum) {
    const AxleTyres p;
    EXPECT_DOUBLE_EQ(temperature_grip(p.thermal, p.thermal.optimal_temp), 1.0);
    EXPECT_NEAR(temperature_grip(p.thermal, p.thermal.optimal_temp + p.thermal.temp_window), 1.0 - p.thermal.grip_loss_at_window, 1e-12);
    EXPECT_DOUBLE_EQ(temperature_grip(p.thermal, -100.0), p.thermal.min_temp_grip);

    EXPECT_DOUBLE_EQ(wear_grip(p.wear, 0.0), 1.0);
    EXPECT_NEAR(wear_grip(p.wear, 1.0), 1.0 - p.wear.grip_loss_linear - p.wear.cliff_loss, 1e-12);
    EXPECT_NEAR(wear_grip(p.wear, p.wear.cliff_start), 1.0 - p.wear.grip_loss_linear * p.wear.cliff_start, 1e-12);

    EXPECT_DOUBLE_EQ(pressure_grip(p.pressure, p.pressure.optimal), 1.0);
    EXPECT_NEAR(pressure_grip(p.pressure, p.pressure.optimal - p.pressure.window), 1.0 - p.pressure.grip_loss_at_window, 1e-12);
    EXPECT_DOUBLE_EQ(pressure_grip(p.pressure, 0.0), p.pressure.min_grip);

    // The cold pressure is at 20 °C, and the default optimum is the default tyre's pressure at 88 °C.
    EXPECT_DOUBLE_EQ(hot_pressure(p.thermal, 20.0), p.thermal.cold_pressure);
    EXPECT_NEAR(hot_pressure(p.thermal, p.thermal.optimal_temp), p.pressure.optimal, 0.05);
}

TEST(TyreCondition, FreshTyresStartAtTheirInitialTemperature) {
    AxleTyres p;
    p.thermal.initial_temp = 60.0;
    const TyreState s = fresh(p);
    EXPECT_DOUBLE_EQ(s.tread_temp, 60.0);
    EXPECT_DOUBLE_EQ(s.carcass_temp, 60.0);
    EXPECT_DOUBLE_EQ(s.wear, 0.0);
    EXPECT_DOUBLE_EQ(s.pressure, hot_pressure(p.thermal, 60.0));
    EXPECT_DOUBLE_EQ(s.grip, temperature_grip(p.thermal, 60.0) * pressure_grip(p.pressure, s.pressure));
    EXPECT_LT(s.grip, 1.0);  // cold, and under its pressure
}

TEST(TyreCondition, AxlesShareTheLoadAndTheForces) {
    const VehicleParams vehicle = create_test_vehicle();
    const TyreConditionParams& c = vehicle.tyre_condition;
    const TrackSegment corner{100.0, 0.01, 1.0, 0.0};
    AxleWork front;
    AxleWork rear;

    // Coasting through a corner: the static weight and the downforce, the lateral force by the weight.
    split(c, vehicle, vehicle.mass, 30.0, 0.0, corner, front, rear);
    const double downforce = 0.5 * constants::air_density * 1.0 * 1.5 * 30.0 * 30.0;
    EXPECT_NEAR(front.load + rear.load, vehicle.mass * constants::g + downforce, 1e-6);
    EXPECT_NEAR(front.load, vehicle.mass * constants::g * 0.55 + downforce * 0.45, 1e-6);
    EXPECT_NEAR(front.force_lat, vehicle.mass * 30.0 * 30.0 * 0.01 * 0.55, 1e-6);
    EXPECT_NEAR(rear.force_lat, vehicle.mass * 30.0 * 30.0 * 0.01 * 0.45, 1e-6);
    EXPECT_EQ(front.force_long, 0.0);

    // Driving: all on the driven (rear) axle, and load moves back.
    AxleWork f2;
    AxleWork r2;
    split(c, vehicle, vehicle.mass, 30.0, 3000.0, corner, f2, r2);
    EXPECT_EQ(f2.force_long, 0.0);
    EXPECT_EQ(r2.force_long, 3000.0);
    EXPECT_NEAR(r2.load - rear.load, 3000.0 * 0.2, 1e-6);
    EXPECT_NEAR(front.load - f2.load, 3000.0 * 0.2, 1e-6);

    // Braking: shared by the bias.
    split(c, vehicle, vehicle.mass, 30.0, -5000.0, corner, f2, r2);
    EXPECT_NEAR(f2.force_long, -5000.0 * 0.65, 1e-9);
    EXPECT_NEAR(r2.force_long, -5000.0 * 0.35, 1e-9);
}

TEST(TyreCondition, SlidingWorkGrowsWithTheGripUsed) {
    const AxleTyres p;
    AxleWork work;
    work.load = 5000.0;
    work.mu_lat = 1.2;
    work.mu_long = 1.25;
    EXPECT_EQ(sliding_power(p, work, 30.0, 1.0), 0.0);

    work.force_lat = 0.3 * 1.2 * 5000.0;
    const double light = sliding_power(p, work, 30.0, 1.0);
    work.force_lat = 0.9 * 1.2 * 5000.0;
    const double hard = sliding_power(p, work, 30.0, 1.0);
    EXPECT_GT(light, 0.0);
    EXPECT_GT(hard, 3.0 * light * 3.0);  // three times the force, and much more slip

    // At the limit the tyre slides at its peak slip: half the axle's force times tan(peak angle) times speed.
    work.force_lat = 1.2 * 5000.0;
    EXPECT_NEAR(sliding_power(p, work, 30.0, 1.0), 0.5 * 6000.0 * std::tan(p.peak_slip_angle) * 30.0, 1e-6);
    EXPECT_NEAR(sliding_power(p, work, 30.0, 2.0), 2.0 * sliding_power(p, work, 30.0, 1.0), 1e-9);
}

TEST(TyreCondition, WorkHeatsAndWearsTheTyreAndThePressureFollows) {
    const VehicleParams vehicle = create_test_vehicle();
    const TyreConditionParams& c = vehicle.tyre_condition;
    const AxleTyres& p = c.rear;
    AxleWork work;
    work.load = 5000.0;
    work.mu_lat = 1.2;
    work.mu_long = 1.25;
    work.force_lat = 0.9 * 1.2 * 5000.0;

    TyreState s = fresh(p);
    const double start = s.tread_temp;
    for (int k = 0; k < 20000; ++k) advance(p, c, work, 30.0, 0.001, s);
    EXPECT_GT(s.tread_temp, start + 10.0);
    EXPECT_GT(s.carcass_temp, start);
    EXPECT_GT(s.wear, 0.0);
    EXPECT_DOUBLE_EQ(s.pressure, hot_pressure(p.thermal, s.carcass_temp));
    EXPECT_NEAR(s.grip, temperature_grip(p.thermal, s.tread_temp) * wear_grip(p.wear, s.wear) * pressure_grip(p.pressure, s.pressure), 1e-12);

    // Rolling without work: no wear, and the tread cools towards the air and the road.
    TyreState hot = s;
    hot.tread_temp = 120.0;
    AxleWork rolling;
    rolling.load = 5000.0;
    const double wear_before = hot.wear;
    for (int k = 0; k < 20000; ++k) advance(p, c, rolling, 30.0, 0.001, hot);
    EXPECT_LT(hot.tread_temp, 110.0);
    EXPECT_DOUBLE_EQ(hot.wear, wear_before);
}

TEST(TyreCondition, UnderInflatedTyresHeatMore) {
    const VehicleParams vehicle = create_test_vehicle();
    const TyreConditionParams& c = vehicle.tyre_condition;
    AxleTyres own = c.rear;
    AxleTyres under = c.rear;
    under.thermal.cold_pressure -= 30.0;
    AxleWork rolling;
    rolling.load = 5000.0;
    TyreState a = fresh(own);
    TyreState b = fresh(under);
    for (int k = 0; k < 60000; ++k) {
        advance(own, c, rolling, 40.0, 0.001, a);
        advance(under, c, rolling, 40.0, 0.001, b);
    }
    EXPECT_GT(b.carcass_temp, a.carcass_temp + 1.0);
    EXPECT_LT(b.pressure, a.pressure);
}
