#include "io/vehicle_loader.hpp"

#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Loads vehicle parameters from a JSON file.
 *
 * Expected JSON structure:
 * {
 *     "name": "Vehicle Name",
 *     "mass": 800.0,
 *     "wheel_radius": 0.33,
 *     "aero": {
 *         "drag_coefficient": 1.0,
 *         "lift_coefficient": -3.0,
 *         "frontal_area": 1.5
 *     },
 *     "drivetrain": {
 *         "engine": {
 *             "rpm": [5000, 10000, 15000],
 *             "torque": [400, 450, 380]
 *         },
 *         "gearbox": {
 *             "ratios": [3.0, 2.0, 1.5, 1.2, 1.0],
 *             "final_drive": 3.5
 *         },
 *         "efficiency": 0.9
 *     },
 *     "max_drive_force": 15000.0,
 *     "max_brake_force": 25000.0
 * }
 *
 * @param path Path to JSON file
 * @return VehicleParams structure with loaded data
 * @throws std::runtime_error if file cannot be read or parsed
 */
VehicleParams load_vehicle(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open vehicle file: " + path);
    }

    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in vehicle file: " + std::string(e.what()));
    }

    VehicleParams vehicle{};

    try {
        // Basic parameters
        vehicle.mass = j.at("mass").get<double>();
        vehicle.wheel_radius = j.at("wheel_radius").get<double>();

        // Aerodynamics
        const auto& aero = j.at("aero");
        vehicle.aero.drag_coefficient = aero.at("drag_coefficient").get<double>();
        vehicle.aero.lift_coefficient = aero.at("lift_coefficient").get<double>();
        vehicle.aero.frontal_area = aero.at("frontal_area").get<double>();

        // Drivetrain
        const auto& dt = j.at("drivetrain");

        // Engine
        const auto& engine = dt.at("engine");
        vehicle.drivetrain.engine.rpm = engine.at("rpm").get<std::vector<double>>();
        vehicle.drivetrain.engine.torque = engine.at("torque").get<std::vector<double>>();

        // Validate engine data
        if (vehicle.drivetrain.engine.rpm.size() != vehicle.drivetrain.engine.torque.size()) {
            throw std::runtime_error("Engine RPM and torque arrays must have same length");
        }
        if (vehicle.drivetrain.engine.rpm.empty()) {
            throw std::runtime_error("Engine data cannot be empty");
        }

        // Gearbox
        const auto& gearbox = dt.at("gearbox");
        vehicle.drivetrain.gearbox.ratios = gearbox.at("ratios").get<std::vector<double>>();
        vehicle.drivetrain.gearbox.final_drive = gearbox.at("final_drive").get<double>();

        if (vehicle.drivetrain.gearbox.ratios.empty()) {
            throw std::runtime_error("Gearbox must have at least one gear ratio");
        }

        // Drivetrain efficiency
        vehicle.drivetrain.efficiency = dt.at("efficiency").get<double>();

        // Force limits
        vehicle.max_drive_force = j.at("max_drive_force").get<double>();
        vehicle.max_brake_force = j.at("max_brake_force").get<double>();

    } catch (const json::out_of_range& e) {
        throw std::runtime_error("Missing required field in vehicle file: " + std::string(e.what()));
    } catch (const json::type_error& e) {
        throw std::runtime_error("Invalid data type in vehicle file: " + std::string(e.what()));
    }

    // Validate ranges
    if (vehicle.mass <= 0.0) {
        throw std::runtime_error("Vehicle mass must be positive");
    }
    if (vehicle.wheel_radius <= 0.0) {
        throw std::runtime_error("Wheel radius must be positive");
    }
    if (vehicle.drivetrain.efficiency <= 0.0 || vehicle.drivetrain.efficiency > 1.0) {
        throw std::runtime_error("Drivetrain efficiency must be between 0 and 1");
    }

    return vehicle;
}
