#include <gtest/gtest.h>
#include <cmath>

#include "physics/tyre_model.hpp"
#include "physics/lateral.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/track.hpp"
#include "core/constants.hpp"

/**
 * @brief A car without downforce, so the grip is easy to work out by hand.
 */
static VehicleParams create_plain_vehicle() {
    VehicleParams vehicle{};
    vehicle.mass = 1000.0;
    vehicle.wheel_radius = 0.3;
    vehicle.aero.drag_coefficient = 0.35;
    vehicle.aero.lift_coefficient = 0.0;
    vehicle.aero.frontal_area = 2.0;
    vehicle.drivetrain.engine.rpm = {1000, 7000};
    vehicle.drivetrain.engine.torque = {300, 300};
    vehicle.drivetrain.gearbox.ratios = {3.0, 2.0, 1.4, 1.0};
    vehicle.drivetrain.gearbox.final_drive = 3.9;
    vehicle.drivetrain.efficiency = 0.9;
    vehicle.max_drive_force = 20000.0;
    vehicle.max_brake_force = 30000.0;
    return vehicle;
}

static CarState at_speed(double v) {
    CarState state{};
    state.v = v;
    state.gear = 2;
    state.engine_rpm = 4000.0;
    return state;
}

static TrackSegment corner(double curvature, double camber = 0.0, double grip = 1.0) {
    return TrackSegment{100.0, curvature, grip, camber};
}

// =============================================================================
// Normal load and lateral demand
// =============================================================================

TEST(TyreModel, FlatRoadLoadIsGravityPlusDownforce) {
    VehicleParams vehicle = create_plain_vehicle();
    EXPECT_NEAR(physics::normal_acceleration(30.0, vehicle, corner(0.02)), constants::g, 1e-12);

    vehicle.aero.lift_coefficient = -2.0;
    const double downforce = 0.5 * constants::air_density * 2.0 * 2.0 * 30.0 * 30.0;
    EXPECT_NEAR(physics::normal_acceleration(30.0, vehicle, corner(0.02)), constants::g + downforce / vehicle.mass, 1e-9);
}

TEST(TyreModel, FlatRoadDemandIsCentripetal) {
    EXPECT_NEAR(physics::lateral_demand(20.0, corner(0.01)), 20.0 * 20.0 * 0.01, 1e-12);
    EXPECT_NEAR(physics::lateral_demand(20.0, corner(-0.01)), 20.0 * 20.0 * 0.01, 1e-12);
    EXPECT_EQ(physics::lateral_demand(20.0, corner(0.0)), 0.0);
}

TEST(TyreModel, BankingTakesDemandAndAddsLoad) {
    const VehicleParams vehicle = create_plain_vehicle();
    const double bank = 0.15;  // rad, into the turn
    const TrackSegment flat = corner(0.02);
    const TrackSegment banked = corner(0.02, bank);

    EXPECT_LT(physics::lateral_demand(25.0, banked), physics::lateral_demand(25.0, flat));
    EXPECT_GT(physics::normal_acceleration(25.0, vehicle, banked), physics::normal_acceleration(25.0, vehicle, flat));
    EXPECT_NEAR(physics::lateral_demand(25.0, banked),
                25.0 * 25.0 * 0.02 * std::cos(bank) - constants::g * std::sin(bank), 1e-12);
}

// =============================================================================
// Friction ellipse
// =============================================================================

TEST(TyreModel, EllipseLeavesAllOnAStraightAndNoneAtTheLimit) {
    const VehicleParams vehicle = create_plain_vehicle();
    EXPECT_DOUBLE_EQ(physics::compute_traction_scale(at_speed(40.0), vehicle, corner(0.0)), 1.0);

    const TrackSegment turn = corner(0.02);
    const double v_limit = physics::lateral::max_speed(turn, vehicle);
    EXPECT_NEAR(physics::compute_traction_scale(at_speed(v_limit), vehicle, turn), 0.0, 1e-6);
    EXPECT_EQ(physics::compute_traction_scale(at_speed(1.1 * v_limit), vehicle, turn), 0.0);

    // Half the lateral grip used leaves √(1 − ¼) of the longitudinal grip, not ½ (a linear circle).
    const double v_half = v_limit / std::sqrt(2.0);
    EXPECT_NEAR(physics::compute_traction_scale(at_speed(v_half), vehicle, turn), std::sqrt(0.75), 1e-9);
}

TEST(TyreModel, LongitudinalGripUsesItsOwnFriction) {
    VehicleParams vehicle = create_plain_vehicle();
    vehicle.tyre.base_grip = 1.1;
    vehicle.tyre.longitudinal_grip = 1.3;
    const double expected = 1.3 * 0.8 * vehicle.mass * constants::g;
    EXPECT_NEAR(physics::longitudinal_grip_force(at_speed(20.0), vehicle, corner(0.0, 0.0, 0.8)), expected, 1e-6);

    // The lateral limit uses the lateral friction.
    EXPECT_NEAR(physics::compute_lateral_acc_limit(at_speed(20.0), vehicle, corner(0.01, 0.0, 0.8)), 1.1 * 0.8 * constants::g, 1e-9);
}

TEST(TyreModel, EngineAndBrakesAreLimitedByGrip) {
    const VehicleParams vehicle = create_plain_vehicle();
    const physics::LongitudinalForces forces = physics::longitudinal_forces(at_speed(20.0), vehicle, corner(0.0));
    ASSERT_GT(forces.brakes, forces.grip);  // 30 kN of brakes against about 9.8 kN of grip

    EXPECT_DOUBLE_EQ(physics::tyre_force(forces, 0.0, 1.0), -forces.grip);
    EXPECT_DOUBLE_EQ(physics::tyre_force(forces, 0.0, 0.1), -0.1 * forces.brakes);
    EXPECT_DOUBLE_EQ(physics::tyre_force(forces, 1.0, 0.0), std::min(forces.engine, forces.grip));
}
