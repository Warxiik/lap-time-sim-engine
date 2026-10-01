#include <gtest/gtest.h>

#include "sim/driver_model.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "core/math.hpp"
#include "models/car_state.hpp"
#include "models/vehicle.hpp"

static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};
    vehicle.mass = 1200.0;
    vehicle.wheel_radius = 0.32;
    vehicle.drivetrain.engine.rpm = {2000, 5000, 8000};
    vehicle.drivetrain.engine.torque = {350, 450, 380};
    vehicle.drivetrain.gearbox.ratios = {3.2, 2.2, 1.6, 1.25, 1.0};
    vehicle.drivetrain.gearbox.final_drive = 3.7;
    vehicle.drivetrain.efficiency = 0.9;
    vehicle.max_drive_force = 12000.0;
    vehicle.max_brake_force = 25000.0;
    return vehicle;
}

/// Torque at the wheels in `gear` at `v` m/s, over the final drive and efficiency (the same in every gear).
static double gear_force(const VehicleParams& vehicle, int gear, double v) {
    const auto& engine = vehicle.drivetrain.engine;
    const double rpm = physics::engine_rpm(v, gear, vehicle);
    return math::interpolate(engine.rpm, engine.torque, rpm) * vehicle.drivetrain.gearbox.ratios[static_cast<size_t>(gear - 1)];
}

/**
 * @test At every speed the driver is in the gear with the most drive force, within the torque curve.
 */
TEST(DriverModel, PicksTheGearWithTheMostDriveForce) {
    const VehicleParams vehicle = create_test_vehicle();
    const DriverModel driver;
    const int gears = static_cast<int>(vehicle.drivetrain.gearbox.ratios.size());
    const double max_rpm = vehicle.drivetrain.engine.rpm.back();

    int previous = 1;
    for (double v = 0.1; v < 85.0; v += 0.25) {
        CarState state{};
        state.v = v;
        state.gear = gears;  // the gear before does not matter
        const int gear = driver.select_gear(state, vehicle);
        ASSERT_GE(gear, 1);
        ASSERT_LE(gear, gears);
        EXPECT_GE(gear, previous) << "at " << v << " m/s";  // never a lower gear at a higher speed
        previous = gear;

        const double rpm = physics::engine_rpm(v, gear, vehicle);
        if (gear < gears) {
            EXPECT_LE(rpm, max_rpm) << "at " << v << " m/s";
        }
        for (int other = 1; other <= gears; ++other) {
            if (physics::engine_rpm(v, other, vehicle) > max_rpm) continue;
            EXPECT_GE(gear_force(vehicle, gear, v), gear_force(vehicle, other, v)) << "gear " << other << " at " << v << " m/s";
        }

        state.gear = 1;
        EXPECT_EQ(driver.select_gear(state, vehicle), gear) << "at " << v << " m/s";
    }
    EXPECT_EQ(previous, gears);  // top gear at the top
}

/**
 * @test From rest every gear is below the curve, so first gear, which multiplies the torque most.
 */
TEST(DriverModel, StartsInFirstGear) {
    const VehicleParams vehicle = create_test_vehicle();
    CarState state{};
    state.v = 0.1;
    state.gear = 3;
    EXPECT_EQ(DriverModel{}.select_gear(state, vehicle), 1);
}

/**
 * @test Faster than the top gear's last RPM, the car stays in top gear.
 */
TEST(DriverModel, StaysInTopGearPastTheCurve) {
    const VehicleParams vehicle = create_test_vehicle();
    CarState state{};
    state.v = 200.0;
    EXPECT_EQ(DriverModel{}.select_gear(state, vehicle), static_cast<int>(vehicle.drivetrain.gearbox.ratios.size()));
}
