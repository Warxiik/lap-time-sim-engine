#include "models/telemetry.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

/**
 * @brief Writes telemetry data to a CSV file for analysis.
 *
 * Output format:
 *   time,distance,velocity,acceleration,gear,rpm,throttle,brake
 *   0.000,0.00,10.00,5.23,1,8500,1.00,0.00
 *   0.001,0.01,10.01,5.21,1,8520,1.00,0.00
 *   ...
 *
 * This format is compatible with common analysis tools:
 *   - Excel/Google Sheets
 *   - Python pandas
 *   - MATLAB
 *   - Motec i2 (with configuration)
 *
 * @param telemetry Telemetry data to write
 * @param path Output file path
 * @throws std::runtime_error if file cannot be written
 */
void write_telemetry_csv(const Telemetry& telemetry, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open telemetry output file: " + path);
    }

    // Write header
    file << "time,distance,velocity,acceleration,gear,rpm,throttle,brake\n";

    // Set precision for floating point values
    file << std::fixed << std::setprecision(4);

    // Write data rows
    const size_t n = telemetry.size();
    for (size_t i = 0; i < n; ++i) {
        file << telemetry.time[i] << ','
             << telemetry.distance[i] << ','
             << telemetry.velocity[i] << ','
             << telemetry.acceleration[i] << ','
             << telemetry.gear[i] << ','
             << telemetry.engine_rpm[i] << ','
             << telemetry.throttle[i] << ','
             << telemetry.brake[i] << '\n';
    }

    if (!file.good()) {
        throw std::runtime_error("Error writing telemetry file: " + path);
    }
}
