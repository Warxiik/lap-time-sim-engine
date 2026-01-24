#include <gtest/gtest.h>
#include <cmath>

#include "physics/longitudinal.hpp"
#include "physics/aero.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/control.hpp"
#include "core/constants.hpp"

/**
 * @brief Creates a test vehicle with known parameters.
 */
static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};

    vehicle.mass = 800.0;           // kg
    vehicle.wheel_radius = 0.33;    // m

    // Aerodynamics
    vehicle.aero.drag_coefficient = 1.0;
    vehicle.aero.lift_coefficient = -3.0;
    vehicle.aero.frontal_area = 1.5;

    // Engine: simplified torque curve
    vehicle.drivetrain.engine.rpm = {5000, 10000, 15000};
    vehicle.drivetrain.engine.torque = {400, 450, 380};  // Nm

    // Gearbox: 5-speed
    vehicle.drivetrain.gearbox.ratios = {3.5, 2.5, 1.8, 1.4, 1.1};
    vehicle.drivetrain.gearbox.final_drive = 3.5;
    vehicle.drivetrain.efficiency = 0.9;

    vehicle.max_drive_force = 15000.0;  // N
    vehicle.max_brake_force = 25000.0;  // N

    return vehicle;
}

/**
 * @brief Creates a test car state.
 */
static CarState create_test_state(double velocity, int gear, double rpm) {
    CarState state{};
    state.s = 0.0;
    state.v = velocity;
    state.a = 0.0;
    state.gear = gear;
    state.engine_rpm = rpm;
    return state;
}

// =============================================================================
// Drive Force Tests
// =============================================================================

/**
 * @test Drive force should be positive when engine produces torque.
 */
TEST(LongitudinalPhysics, DriveForceIsPositive) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(30.0, 3, 10000);

    double drive_force = physics::longitudinal::max_drive_force(vehicle, state);

    EXPECT_GT(drive_force, 0.0);
}

/**
 * @test Lower gears should produce more drive force (higher torque multiplication).
 */
TEST(LongitudinalPhysics, LowerGearsProduceMoreForce) {
    VehicleParams vehicle = create_test_vehicle();

    CarState state_gear1 = create_test_state(20.0, 1, 10000);
    CarState state_gear5 = create_test_state(20.0, 5, 10000);

    double force_gear1 = physics::longitudinal::max_drive_force(vehicle, state_gear1);
    double force_gear5 = physics::longitudinal::max_drive_force(vehicle, state_gear5);

    EXPECT_GT(force_gear1, force_gear5);
}

/**
 * @test Invalid gear should return zero drive force.
 */
TEST(LongitudinalPhysics, InvalidGearReturnsZeroForce) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(30.0, 0, 10000);  // Gear 0 is invalid

    double drive_force = physics::longitudinal::max_drive_force(vehicle, state);

    EXPECT_EQ(drive_force, 0.0);
}

// =============================================================================
// Brake Force Tests
// =============================================================================

/**
 * @test Brake force should equal max brake force (simplified model).
 */
TEST(LongitudinalPhysics, BrakeForceEqualsMax) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(50.0, 3, 10000);

    double brake_force = physics::longitudinal::max_brake_force(vehicle, state);

    EXPECT_EQ(brake_force, vehicle.max_brake_force);
}

// =============================================================================
// Net Force Tests
// =============================================================================

/**
 * @test Full throttle with no brake should give positive net force.
 */
TEST(LongitudinalPhysics, FullThrottleGivesPositiveNetForce) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(30.0, 3, 10000);
    ControlInput control{1.0, 0.0};  // Full throttle, no brake

    double drag = physics::aero::drag_force(vehicle, state);
    double net = physics::longitudinal::net_force(vehicle, state, control, drag);

    // At moderate speed, drive force should exceed drag
    EXPECT_GT(net, 0.0);
}

/**
 * @test Full brake with no throttle should give negative net force.
 */
TEST(LongitudinalPhysics, FullBrakeGivesNegativeNetForce) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(30.0, 3, 10000);
    ControlInput control{0.0, 1.0};  // No throttle, full brake

    double drag = physics::aero::drag_force(vehicle, state);
    double net = physics::longitudinal::net_force(vehicle, state, control, drag);

    EXPECT_LT(net, 0.0);
}

/**
 * @test Zero inputs should result in negative net force (drag only).
 */
TEST(LongitudinalPhysics, NoInputsGivesDragOnly) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(50.0, 3, 10000);
    ControlInput control{0.0, 0.0};  // Coasting

    double drag = physics::aero::drag_force(vehicle, state);
    double net = physics::longitudinal::net_force(vehicle, state, control, drag);

    // Net force should equal negative drag (deceleration)
    EXPECT_DOUBLE_EQ(net, -drag);
}

// =============================================================================
// Aerodynamic Tests
// =============================================================================

/**
 * @test Drag should increase with velocity squared.
 */
TEST(AerodynamicPhysics, DragIncreasesWithVelocitySquared) {
    VehicleParams vehicle = create_test_vehicle();

    CarState state_slow = create_test_state(25.0, 3, 10000);
    CarState state_fast = create_test_state(50.0, 3, 10000);  // 2x speed

    double drag_slow = physics::aero::drag_force(vehicle, state_slow);
    double drag_fast = physics::aero::drag_force(vehicle, state_fast);

    // At 2x speed, drag should be ~4x (v² relationship)
    double ratio = drag_fast / drag_slow;
    EXPECT_NEAR(ratio, 4.0, 0.01);
}

/**
 * @test Downforce should increase with velocity squared.
 */
TEST(AerodynamicPhysics, DownforceIncreasesWithVelocitySquared) {
    VehicleParams vehicle = create_test_vehicle();

    CarState state_slow = create_test_state(25.0, 3, 10000);
    CarState state_fast = create_test_state(50.0, 3, 10000);

    double df_slow = physics::aero::downforce(vehicle, state_slow);
    double df_fast = physics::aero::downforce(vehicle, state_fast);

    // At 2x speed, downforce should be ~4x
    double ratio = df_fast / df_slow;
    EXPECT_NEAR(ratio, 4.0, 0.01);
}

/**
 * @test Downforce should be positive for negative lift coefficient.
 */
TEST(AerodynamicPhysics, DownforceIsPositiveForNegativeCl) {
    VehicleParams vehicle = create_test_vehicle();
    // vehicle.aero.lift_coefficient is -3.0 (negative = downforce)

    CarState state = create_test_state(50.0, 3, 10000);

    double df = physics::aero::downforce(vehicle, state);

    EXPECT_GT(df, 0.0);
}

/**
 * @test Drag values should be in reasonable range for F1-like car.
 */
TEST(AerodynamicPhysics, DragInReasonableRange) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(83.3, 3, 10000);  // ~300 km/h

    double drag = physics::aero::drag_force(vehicle, state);

    // At 300 km/h, drag should be roughly 5000-10000 N for F1
    EXPECT_GT(drag, 3000.0);
    EXPECT_LT(drag, 15000.0);
}
