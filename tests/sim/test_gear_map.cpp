#include <gtest/gtest.h>

#include "sim/gear_map.hpp"
#include "sim/driver_model.hpp"
#include "physics/vehicle_dynamics.hpp"
#include "models/car_state.hpp"
#include "models/vehicle.hpp"

static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};
    vehicle.mass = 800.0;
    vehicle.wheel_radius = 0.33;
    vehicle.drivetrain.engine.rpm = {5000, 7000, 9000, 11000, 13000, 15000};
    vehicle.drivetrain.engine.torque = {420, 500, 540, 530, 490, 440};
    vehicle.drivetrain.gearbox.ratios = {3.2, 2.5, 2.0, 1.6, 1.3, 1.1, 1.0, 0.9};
    vehicle.drivetrain.gearbox.final_drive = 3.0;
    vehicle.drivetrain.efficiency = 0.92;
    vehicle.max_drive_force = 20000.0;
    vehicle.max_brake_force = 30000.0;
    return vehicle;
}

static int select(const VehicleParams& vehicle, double v) {
    CarState state{};
    state.v = v;
    return DriverModel{}.select_gear(state, vehicle);
}

/**
 * @test The map gives the gear the driver's own selection does, at every speed from rest to past the top.
 */
TEST(GearMap, AgreesWithTheDriversSelection) {
    for (const bool flat : {false, true}) {
        VehicleParams vehicle = create_test_vehicle();
        if (flat) {
            vehicle.drivetrain.engine.torque = {500, 500, 500, 500, 500, 500};  // every gear ties below its limit
        }
        const GearMap map(vehicle);
        const int gears = static_cast<int>(vehicle.drivetrain.gearbox.ratios.size());
        ASSERT_EQ(static_cast<int>(map.shift_speeds().size()), gears - 1);
        for (size_t k = 1; k < map.shift_speeds().size(); ++k) {
            ASSERT_GT(map.shift_speeds()[k], map.shift_speeds()[k - 1]);
        }
        for (double v = 0.0; v < 150.0; v += 0.0137) {
            ASSERT_EQ(map.gear_at(v), select(vehicle, v)) << "at " << v << " m/s" << (flat ? ", flat curve" : "");
        }
        // Either side of each shift speed.
        for (double s : map.shift_speeds()) {
            EXPECT_EQ(map.gear_at(s * (1.0 - 1e-9)), select(vehicle, s * (1.0 - 1e-9)));
            EXPECT_EQ(map.gear_at(s * (1.0 + 1e-9)), select(vehicle, s * (1.0 + 1e-9)));
        }
    }
}

/**
 * @test With a flat torque curve the car holds each gear to the end of the curve.
 */
TEST(GearMap, FlatCurveShiftsAtTheLastRpm) {
    VehicleParams vehicle = create_test_vehicle();
    vehicle.drivetrain.engine.torque = {500, 500, 500, 500, 500, 500};
    const GearMap map(vehicle);
    for (size_t k = 0; k < map.shift_speeds().size(); ++k) {
        const int gear = static_cast<int>(k) + 1;
        EXPECT_NEAR(physics::engine_rpm(map.shift_speeds()[k], gear, vehicle), 15000.0, 1e-6) << "out of gear " << gear;
    }
}
