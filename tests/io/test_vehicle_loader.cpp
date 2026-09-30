#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>

#include "io/vehicle_loader.hpp"

/**
 * @brief Returns a valid vehicle JSON string for testing.
 */
static std::string valid_vehicle_json() {
    return R"({
        "name": "Test Car",
        "mass": 800.0,
        "wheel_radius": 0.33,
        "aero": {
            "drag_coefficient": 1.0,
            "lift_coefficient": -3.0,
            "frontal_area": 1.5
        },
        "drivetrain": {
            "engine": {
                "rpm": [5000, 10000, 15000],
                "torque": [400, 450, 380]
            },
            "gearbox": {
                "ratios": [3.0, 2.0, 1.5, 1.2, 1.0],
                "final_drive": 3.5
            },
            "efficiency": 0.9
        },
        "max_drive_force": 15000.0,
        "max_brake_force": 25000.0
    })";
}

static std::string write_temp_json(const std::string& content) {
    std::string path = "test_vehicle_tmp.json";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

static void remove_temp(const std::string& path) {
    std::remove(path.c_str());
}

// =============================================================================
// Vehicle Loader Tests
// =============================================================================

TEST(VehicleLoader, LoadsValidVehicle) {
    auto path = write_temp_json(valid_vehicle_json());
    VehicleParams v = load_vehicle(path);
    remove_temp(path);

    EXPECT_DOUBLE_EQ(v.mass, 800.0);
    EXPECT_DOUBLE_EQ(v.wheel_radius, 0.33);
    EXPECT_DOUBLE_EQ(v.aero.drag_coefficient, 1.0);
    EXPECT_DOUBLE_EQ(v.aero.lift_coefficient, -3.0);
    EXPECT_DOUBLE_EQ(v.aero.frontal_area, 1.5);
    ASSERT_EQ(v.drivetrain.engine.rpm.size(), 3u);
    ASSERT_EQ(v.drivetrain.gearbox.ratios.size(), 5u);
    EXPECT_DOUBLE_EQ(v.drivetrain.gearbox.final_drive, 3.5);
    EXPECT_DOUBLE_EQ(v.drivetrain.efficiency, 0.9);
    EXPECT_DOUBLE_EQ(v.max_drive_force, 15000.0);
    EXPECT_DOUBLE_EQ(v.max_brake_force, 25000.0);
    // No "tyre" block: the track's grip alone.
    EXPECT_DOUBLE_EQ(v.tyre.base_grip, 1.0);
    EXPECT_DOUBLE_EQ(v.tyre.longitudinal_grip, 1.0);
}

/**
 * @brief The valid vehicle with a "tyre" block appended.
 */
static std::string vehicle_json_with_tyre(const std::string& tyre_block) {
    std::string json = valid_vehicle_json();
    const size_t end = json.rfind('}');
    return json.substr(0, end) + ", \"tyre\": " + tyre_block + "}";
}

TEST(VehicleLoader, LoadsTyreGrip) {
    auto path = write_temp_json(vehicle_json_with_tyre(R"({ "base_grip": 1.6, "longitudinal_grip": 1.7 })"));
    VehicleParams v = load_vehicle(path);
    remove_temp(path);

    EXPECT_DOUBLE_EQ(v.tyre.base_grip, 1.6);
    EXPECT_DOUBLE_EQ(v.tyre.longitudinal_grip, 1.7);
}

TEST(VehicleLoader, TyreFieldsDefaultToOne) {
    auto path = write_temp_json(vehicle_json_with_tyre(R"({ "base_grip": 1.3 })"));
    VehicleParams v = load_vehicle(path);
    remove_temp(path);

    EXPECT_DOUBLE_EQ(v.tyre.base_grip, 1.3);
    EXPECT_DOUBLE_EQ(v.tyre.longitudinal_grip, 1.0);
}

TEST(VehicleLoader, ThrowsOnNonPositiveTyreGrip) {
    auto path = write_temp_json(vehicle_json_with_tyre(R"({ "base_grip": 0.0 })"));
    EXPECT_THROW(load_vehicle(path), std::runtime_error);
    remove_temp(path);
}

TEST(VehicleLoader, ThrowsOnMissingFile) {
    EXPECT_THROW(load_vehicle("nonexistent_file.json"), std::runtime_error);
}

TEST(VehicleLoader, ThrowsOnInvalidJson) {
    auto path = write_temp_json("{ not valid json }}}");
    EXPECT_THROW(load_vehicle(path), std::runtime_error);
    remove_temp(path);
}

TEST(VehicleLoader, ThrowsOnMissingMass) {
    auto path = write_temp_json(R"({
        "wheel_radius": 0.33,
        "aero": { "drag_coefficient": 1.0, "lift_coefficient": -3.0, "frontal_area": 1.5 },
        "drivetrain": {
            "engine": { "rpm": [5000], "torque": [400] },
            "gearbox": { "ratios": [3.0], "final_drive": 3.5 },
            "efficiency": 0.9
        },
        "max_drive_force": 15000.0,
        "max_brake_force": 25000.0
    })");
    EXPECT_THROW(load_vehicle(path), std::runtime_error);
    remove_temp(path);
}

TEST(VehicleLoader, ThrowsOnNegativeMass) {
    auto path = write_temp_json(R"({
        "mass": -100.0,
        "wheel_radius": 0.33,
        "aero": { "drag_coefficient": 1.0, "lift_coefficient": -3.0, "frontal_area": 1.5 },
        "drivetrain": {
            "engine": { "rpm": [5000], "torque": [400] },
            "gearbox": { "ratios": [3.0], "final_drive": 3.5 },
            "efficiency": 0.9
        },
        "max_drive_force": 15000.0,
        "max_brake_force": 25000.0
    })");
    EXPECT_THROW(load_vehicle(path), std::runtime_error);
    remove_temp(path);
}

TEST(VehicleLoader, ThrowsOnMismatchedEngineArrays) {
    auto path = write_temp_json(R"({
        "mass": 800.0,
        "wheel_radius": 0.33,
        "aero": { "drag_coefficient": 1.0, "lift_coefficient": -3.0, "frontal_area": 1.5 },
        "drivetrain": {
            "engine": { "rpm": [5000, 10000], "torque": [400] },
            "gearbox": { "ratios": [3.0], "final_drive": 3.5 },
            "efficiency": 0.9
        },
        "max_drive_force": 15000.0,
        "max_brake_force": 25000.0
    })");
    EXPECT_THROW(load_vehicle(path), std::runtime_error);
    remove_temp(path);
}

TEST(VehicleLoader, ThrowsOnInvalidEfficiency) {
    auto path = write_temp_json(R"({
        "mass": 800.0,
        "wheel_radius": 0.33,
        "aero": { "drag_coefficient": 1.0, "lift_coefficient": -3.0, "frontal_area": 1.5 },
        "drivetrain": {
            "engine": { "rpm": [5000], "torque": [400] },
            "gearbox": { "ratios": [3.0], "final_drive": 3.5 },
            "efficiency": 1.5
        },
        "max_drive_force": 15000.0,
        "max_brake_force": 25000.0
    })");
    EXPECT_THROW(load_vehicle(path), std::runtime_error);
    remove_temp(path);
}
