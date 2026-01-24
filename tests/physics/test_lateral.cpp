#include <gtest/gtest.h>
#include <cmath>

#include "physics/lateral.hpp"
#include "physics/tyre_model.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/track.hpp"
#include "core/constants.hpp"

/**
 * @brief Creates a test vehicle with known parameters.
 *
 * This helper function creates a vehicle with realistic but simplified
 * parameters for testing. Using fixed values ensures test reproducibility.
 */
static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};

    vehicle.mass = 800.0;           // kg (F1-like)
    vehicle.wheel_radius = 0.33;    // m

    // Aerodynamics: high downforce configuration
    vehicle.aero.drag_coefficient = 1.0;
    vehicle.aero.lift_coefficient = -3.0;  // Negative = downforce
    vehicle.aero.frontal_area = 1.5;       // m²

    // Drivetrain (not used in lateral tests, but needed for struct)
    vehicle.drivetrain.engine.rpm = {5000, 10000, 15000};
    vehicle.drivetrain.engine.torque = {400, 450, 380};
    vehicle.drivetrain.gearbox.ratios = {3.0, 2.0, 1.5, 1.2, 1.0};
    vehicle.drivetrain.gearbox.final_drive = 3.5;
    vehicle.drivetrain.efficiency = 0.9;

    vehicle.max_drive_force = 15000.0;
    vehicle.max_brake_force = 25000.0;

    return vehicle;
}

/**
 * @brief Creates a test car state at given velocity.
 */
static CarState create_test_state(double velocity) {
    CarState state{};
    state.s = 0.0;
    state.v = velocity;
    state.a = 0.0;
    state.gear = 3;
    state.engine_rpm = 10000;
    return state;
}

// =============================================================================
// Lateral Physics Tests
// =============================================================================

/**
 * @test Straight sections should have effectively unlimited cornering speed.
 */
TEST(LateralPhysics, StraightSectionHasUnlimitedSpeed) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(50.0);  // 50 m/s

    TrackSegment straight{};
    straight.length = 100.0;
    straight.curvature = 0.0;  // Straight
    straight.grip = 1.0;
    straight.camber = 0.0;

    double v_max = physics::lateral::max_speed(straight, vehicle, state);

    // Should be very large (essentially unlimited)
    EXPECT_GT(v_max, 1000.0);
}

/**
 * @test Tighter corners (higher curvature) should have lower max speed.
 */
TEST(LateralPhysics, TighterCornerHasLowerMaxSpeed) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(50.0);

    TrackSegment gentle_curve{};
    gentle_curve.length = 100.0;
    gentle_curve.curvature = 0.01;  // 100m radius
    gentle_curve.grip = 1.0;
    gentle_curve.camber = 0.0;

    TrackSegment tight_curve{};
    tight_curve.length = 100.0;
    tight_curve.curvature = 0.02;  // 50m radius
    tight_curve.grip = 1.0;
    tight_curve.camber = 0.0;

    double v_max_gentle = physics::lateral::max_speed(gentle_curve, vehicle, state);
    double v_max_tight = physics::lateral::max_speed(tight_curve, vehicle, state);

    EXPECT_GT(v_max_gentle, v_max_tight);
}

/**
 * @test Higher grip surface should allow higher cornering speed.
 */
TEST(LateralPhysics, HigherGripAllowsHigherSpeed) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(50.0);

    TrackSegment low_grip{};
    low_grip.length = 100.0;
    low_grip.curvature = 0.01;
    low_grip.grip = 0.7;  // Wet track
    low_grip.camber = 0.0;

    TrackSegment high_grip{};
    high_grip.length = 100.0;
    high_grip.curvature = 0.01;
    high_grip.grip = 1.2;  // Sticky surface
    high_grip.camber = 0.0;

    double v_max_low = physics::lateral::max_speed(low_grip, vehicle, state);
    double v_max_high = physics::lateral::max_speed(high_grip, vehicle, state);

    EXPECT_GT(v_max_high, v_max_low);
}

/**
 * @test Lateral acceleration limit should increase with downforce.
 *
 * At higher speeds, downforce increases, which should increase the
 * lateral acceleration limit (a_lat_max = μ * (mg + F_down) / m).
 */
TEST(LateralPhysics, DownforceIncreasesLateralLimit) {
    VehicleParams vehicle = create_test_vehicle();

    TrackSegment segment{};
    segment.length = 100.0;
    segment.curvature = 0.01;
    segment.grip = 1.0;
    segment.camber = 0.0;

    // Low speed = low downforce
    CarState low_speed_state = create_test_state(20.0);
    double a_lat_low = physics::compute_lateral_acc_limit(low_speed_state, vehicle, segment);

    // High speed = high downforce
    CarState high_speed_state = create_test_state(80.0);
    double a_lat_high = physics::compute_lateral_acc_limit(high_speed_state, vehicle, segment);

    // Higher speed should give higher lateral acceleration limit
    EXPECT_GT(a_lat_high, a_lat_low);
}

/**
 * @test Basic sanity check: lateral acceleration should be reasonable.
 *
 * At 50 m/s with moderate aero (Cl=-3.0), the car generates roughly
 * equal downforce to weight, giving ~1.8-2g lateral capability.
 * Higher speeds or more aggressive aero would give higher values.
 */
TEST(LateralPhysics, LateralAccelerationInReasonableRange) {
    VehicleParams vehicle = create_test_vehicle();
    CarState state = create_test_state(50.0);  // ~180 km/h

    TrackSegment segment{};
    segment.length = 100.0;
    segment.curvature = 0.01;
    segment.grip = 1.0;
    segment.camber = 0.0;

    double a_lat_max = physics::compute_lateral_acc_limit(state, vehicle, segment);

    // At 50 m/s with Cl=-3.0:
    //   Downforce ≈ 6,890 N (about 0.88x weight)
    //   Expected a_lat ≈ 1.88g ≈ 18.4 m/s²
    // Should be between 1.5g and 4g for this configuration
    EXPECT_GT(a_lat_max, 1.5 * constants::g);
    EXPECT_LT(a_lat_max, 4.0 * constants::g);
}
