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

    // Without a lateral load transfer both tyres of an axle carry half of it.
    EXPECT_DOUBLE_EQ(front.right_share, 0.5);
    EXPECT_DOUBLE_EQ(rear.right_share, 0.5);
}

TEST(TyreCondition, CornersMoveLoadOntoTheOutsideTyres) {
    VehicleParams vehicle = create_test_vehicle();
    TyreConditionParams& c = vehicle.tyre_condition;
    c.front.lateral_transfer = 0.2;
    c.rear.lateral_transfer = 0.1;
    AxleWork front;
    AxleWork rear;

    // A left-hander loads the right-hand tyres: 0.2 N on the front's for every newton of the car's lateral force.
    split(c, vehicle, vehicle.mass, 30.0, 0.0, TrackSegment{100.0, 0.01, 1.0, 0.0}, front, rear);
    const double lateral = vehicle.mass * 30.0 * 30.0 * 0.01;
    EXPECT_NEAR(front.right_share, 0.5 + 0.2 * lateral / front.load, 1e-12);
    EXPECT_NEAR(rear.right_share, 0.5 + 0.1 * lateral / rear.load, 1e-12);

    // A right-hander mirrors it, a straight leaves both alike, and no tyre carries more than its axle.
    AxleWork f2;
    AxleWork r2;
    split(c, vehicle, vehicle.mass, 30.0, 0.0, TrackSegment{100.0, -0.01, 1.0, 0.0}, f2, r2);
    EXPECT_NEAR(f2.right_share, 1.0 - front.right_share, 1e-12);
    split(c, vehicle, vehicle.mass, 30.0, 0.0, TrackSegment{100.0, 0.0, 1.0, 0.0}, f2, r2);
    EXPECT_DOUBLE_EQ(f2.right_share, 0.5);
    split(c, vehicle, vehicle.mass, 80.0, 0.0, TrackSegment{100.0, 0.05, 1.0, 0.0}, f2, r2);
    EXPECT_DOUBLE_EQ(f2.right_share, 1.0);

    // Banking: a slow car on a banked left-hander leans on its lower, left-hand tyres.
    split(c, vehicle, vehicle.mass, 5.0, 0.0, TrackSegment{100.0, 0.01, 1.0, 0.2}, f2, r2);
    EXPECT_LT(f2.right_share, 0.5);
}

TEST(TyreCondition, SlidingWorkGrowsWithTheGripUsed) {
    const AxleTyres p;
    AxleWork work;
    work.load = 5000.0;
    work.mu_lat = 1.2;
    work.mu_long = 1.25;
    EXPECT_EQ(sliding_power(p, work, 1.0, 30.0), 0.0);

    work.force_lat = 0.3 * 1.2 * 5000.0;
    const double light = sliding_power(p, work, 1.0, 30.0);
    work.force_lat = 0.9 * 1.2 * 5000.0;
    const double hard = sliding_power(p, work, 1.0, 30.0);
    EXPECT_GT(light, 0.0);
    EXPECT_GT(hard, 3.0 * light * 3.0);  // three times the force, and much more slip

    // At the limit the tyres slide at their peak slip: the axle's force times tan(peak angle) times speed.
    work.force_lat = 1.2 * 5000.0;
    EXPECT_NEAR(sliding_power(p, work, 1.0, 30.0), 6000.0 * std::tan(p.peak_slip_angle) * 30.0, 1e-6);
    AxleTyres doubled = p;
    doubled.sliding_work = 2.0;
    EXPECT_NEAR(sliding_power(doubled, work, 1.0, 30.0), 2.0 * sliding_power(p, work, 1.0, 30.0), 1e-9);
    // Tyres with less grip use more of it for the same force: half the grip, and half the force is the limit.
    work.force_lat = 0.5 * 1.2 * 5000.0;
    EXPECT_NEAR(sliding_power(p, work, 0.5, 30.0), 3000.0 * std::tan(p.peak_slip_angle) * 30.0, 1e-6);
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
    TyreState other = s;
    const double start = s.tread_temp;
    for (int k = 0; k < 20000; ++k) advance(p, c, work, 30.0, 0.001, s, other);
    EXPECT_EQ(s.tread_temp, other.tread_temp);  // half the load and the work each
    EXPECT_EQ(s.wear, other.wear);
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
    TyreState hot_other = hot;
    for (int k = 0; k < 20000; ++k) advance(p, c, rolling, 30.0, 0.001, hot, hot_other);
    EXPECT_LT(hot.tread_temp, 110.0);
    EXPECT_DOUBLE_EQ(hot.wear, wear_before);
}

TEST(TyreCondition, EachTyreTakesTheWorkOfItsLoadAndGrip) {
    const VehicleParams vehicle = create_test_vehicle();
    const TyreConditionParams& c = vehicle.tyre_condition;
    const AxleTyres& p = c.front;
    AxleWork work;
    work.load = 5000.0;
    work.mu_lat = 1.2;
    work.mu_long = 1.25;
    work.force_lat = 0.8 * 1.2 * 5000.0;
    work.right_share = 0.75;

    // The loaded outside tyre does three times the inside one's work, and gets hotter and wears faster.
    TyreState left = fresh(p);
    TyreState right = left;
    for (int k = 0; k < 20000; ++k) advance(p, c, work, 30.0, 0.001, left, right);
    EXPECT_GT(right.tread_temp, left.tread_temp + 5.0);
    EXPECT_GT(right.wear, 2.0 * left.wear);

    // One step from equal tyres splits the axle's sliding power 1 : 3, and the total is the axle's.
    TyreState l = fresh(p);
    TyreState r = l;
    l.tread_temp = r.tread_temp = l.carcass_temp = r.carcass_temp = p.thermal.optimal_temp;
    l.grip = r.grip = 1.0;
    const TyreState l0 = l;
    const TyreState r0 = r;
    advance(p, c, work, 30.0, 0.01, l, r);
    TyreState l_alone = l0;
    TyreState r_alone = r0;
    const double total = sliding_power(p, work, 1.0, 30.0);
    advance_tyre(p, c, 0.25 * work.load, 0.25 * total, 30.0, 0.01, l_alone);
    advance_tyre(p, c, 0.75 * work.load, 0.75 * total, 30.0, 0.01, r_alone);
    EXPECT_DOUBLE_EQ(l.wear, l_alone.wear);
    EXPECT_DOUBLE_EQ(r.wear, r_alone.wear);
    EXPECT_DOUBLE_EQ(r.tread_temp, r_alone.tread_temp);

    // A tyre with less grip gives, and works, less at the same slip.
    TyreState weak = l0;
    TyreState strong = r0;
    weak.grip = 0.5;
    work.right_share = 0.5;
    advance(p, c, work, 30.0, 0.01, weak, strong);
    EXPECT_LT(weak.wear, strong.wear);

    // Load sensitivity: the loaded tyre's friction falls off, so it gives (and works) less than its share of
    // the load, and the axle as a whole grips less, using more of it.
    AxleTyres sensitive = p;
    sensitive.load_sensitivity = -0.1;
    sensitive.reference_load = 2500.0;
    EXPECT_DOUBLE_EQ(load_factor(sensitive, 2500.0), 1.0);
    EXPECT_NEAR(load_factor(sensitive, 3750.0), 0.95, 1e-12);
    EXPECT_DOUBLE_EQ(load_factor(sensitive, 1e9), 0.3);
    EXPECT_DOUBLE_EQ(load_factor(p, 1e9), 1.0);
    work.right_share = 0.75;
    TyreState ls = l0;
    TyreState rs = r0;
    advance(sensitive, c, work, 30.0, 0.01, ls, rs);
    const double inside = 1.05 * 1250.0;   // friction 1 + 0.1 · (1 − 1250 / 2500) on 1250 N
    const double outside = 0.95 * 3750.0;  // and 1 − 0.1 · (3750 / 2500 − 1) on 3750 N
    const double felt = (inside + outside) / work.load;
    const double total_sensitive = sliding_power(sensitive, work, felt, 30.0);
    TyreState ls_alone = l0;
    TyreState rs_alone = r0;
    advance_tyre(sensitive, c, 1250.0, inside / (inside + outside) * total_sensitive, 30.0, 0.01, ls_alone);
    advance_tyre(sensitive, c, 3750.0, outside / (inside + outside) * total_sensitive, 30.0, 0.01, rs_alone);
    EXPECT_DOUBLE_EQ(ls.wear, ls_alone.wear);
    EXPECT_DOUBLE_EQ(rs.wear, rs_alone.wear);
    EXPECT_LT(rs.wear / ls.wear, r.wear / l.wear);
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
        advance_tyre(own, c, 0.5 * rolling.load, 0.0, 40.0, 0.001, a);
        advance_tyre(under, c, 0.5 * rolling.load, 0.0, 40.0, 0.001, b);
    }
    EXPECT_GT(b.carcass_temp, a.carcass_temp + 1.0);
    EXPECT_LT(b.pressure, a.pressure);
}
