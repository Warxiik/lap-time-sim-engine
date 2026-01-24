#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
#include <numeric>
#include <vector>
#include <filesystem>
#include <limits>

#include "sim/simulator.hpp"
#include "sim/sim_config.hpp"
#include "io/track_loader.hpp"
#include "io/vehicle_loader.hpp"

namespace fs = std::filesystem;

/**
 * @brief Scans a directory for files with a specific extension.
 *
 * @param directory Path to directory
 * @param extension File extension to filter (e.g., ".csv")
 * @return Vector of file paths
 */
std::vector<fs::path> scan_directory(const fs::path& directory, const std::string& extension) {
    std::vector<fs::path> files;

    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        return files;
    }

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == extension) {
            files.push_back(entry.path());
        }
    }

    // Sort alphabetically for consistent ordering
    std::sort(files.begin(), files.end());
    return files;
}

/**
 * @brief Displays a numbered list and gets user selection.
 *
 * @param title Menu title
 * @param items List of items to display
 * @return Selected index (0-based), or -1 if invalid
 */
int select_from_list(const std::string& title, const std::vector<fs::path>& items) {
    if (items.empty()) {
        std::cout << "No " << title << " found.\n";
        return -1;
    }

    std::cout << "\n=== " << title << " ===\n";
    for (size_t i = 0; i < items.size(); ++i) {
        std::cout << "  [" << (i + 1) << "] " << items[i].stem().string() << "\n";
    }

    std::cout << "\nSelect " << title << " (1-" << items.size() << "): ";

    int choice;
    std::cin >> choice;

    if (std::cin.fail() || choice < 1 || choice > static_cast<int>(items.size())) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return -1;
    }

    return choice - 1;  // Convert to 0-based index
}

/**
 * @brief Formats a lap time as MM:SS.mmm
 */
std::string format_lap_time(double seconds) {
    const int minutes = static_cast<int>(seconds) / 60;
    const double remaining = seconds - (minutes * 60);

    std::ostringstream oss;
    oss << minutes << ":"
        << std::setfill('0') << std::setw(6)
        << std::fixed << std::setprecision(3) << remaining;
    return oss.str();
}

/**
 * @brief Prints telemetry statistics.
 */
void print_telemetry_stats(const Telemetry& telemetry) {
    if (telemetry.size() == 0) {
        std::cout << "No telemetry data recorded.\n";
        return;
    }

    const double max_speed = *std::max_element(telemetry.velocity.begin(), telemetry.velocity.end());
    const double min_speed = *std::min_element(telemetry.velocity.begin(), telemetry.velocity.end());
    const double avg_speed = std::accumulate(telemetry.velocity.begin(), telemetry.velocity.end(), 0.0)
                       / static_cast<double>(telemetry.velocity.size());

    const double max_accel = *std::max_element(telemetry.acceleration.begin(), telemetry.acceleration.end());
    const double max_decel = *std::min_element(telemetry.acceleration.begin(), telemetry.acceleration.end());

    const int max_gear = *std::max_element(telemetry.gear.begin(), telemetry.gear.end());

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\n=== Telemetry Summary ===\n";
    std::cout << "Speed:\n";
    std::cout << "  Maximum:  " << max_speed * 3.6 << " km/h (" << max_speed << " m/s)\n";
    std::cout << "  Minimum:  " << min_speed * 3.6 << " km/h\n";
    std::cout << "  Average:  " << avg_speed * 3.6 << " km/h\n";
    std::cout << "\nAcceleration:\n";
    std::cout << "  Max accel: " << max_accel << " m/s^2 (" << max_accel / 9.81 << " g)\n";
    std::cout << "  Max decel: " << max_decel << " m/s^2 (" << max_decel / 9.81 << " g)\n";
    std::cout << "\nGearbox:\n";
    std::cout << "  Top gear used: " << max_gear << "\n";
    std::cout << "\nData points: " << telemetry.size() << "\n";
}

/**
 * @brief Finds the data directory by checking multiple possible locations.
 *
 * Searches in order:
 *   1. ./data (current working directory)
 *   2. ../data (parent directory, for running from build folder)
 *   3. ../../data (grandparent, for nested build folders)
 *
 * @return Path to data directory, or empty path if not found
 */
fs::path find_data_directory() {
    const std::vector<fs::path> search_paths = {
        "data",
        "../data",
        "../../data",
        "../../../data"
    };

    for (const auto& path : search_paths) {
        if (fs::exists(path) && fs::is_directory(path)) {
            return fs::canonical(path);
        }
    }

    return {};
}

int main() {
    std::cout << "========================================\n";
    std::cout << "     LAP TIME SIMULATION ENGINE\n";
    std::cout << "========================================\n";

    // Find data directory
    fs::path data_dir = find_data_directory();
    if (data_dir.empty()) {
        std::cerr << "Error: Could not find 'data' directory.\n";
        std::cerr << "Make sure to run from the project root or set working directory.\n";
        return 1;
    }

    const fs::path tracks_dir = data_dir / "tracks";
    const fs::path vehicles_dir = data_dir / "vehicles";

    // Scan for available tracks and vehicles
    const std::vector<fs::path> tracks = scan_directory(tracks_dir, ".csv");
    const std::vector<fs::path> vehicles = scan_directory(vehicles_dir, ".json");

    if (tracks.empty()) {
        std::cerr << "Error: No track files found in " << tracks_dir << "\n";
        std::cerr << "Add .csv track files to continue.\n";
        return 1;
    }

    if (vehicles.empty()) {
        std::cerr << "Error: No vehicle files found in " << vehicles_dir << "\n";
        std::cerr << "Add .json vehicle files to continue.\n";
        return 1;
    }

    // Let user select track
    const int track_idx = select_from_list("Available Tracks", tracks);
    if (track_idx < 0) {
        std::cerr << "Invalid track selection.\n";
        return 1;
    }

    // Let user select vehicle
    const int vehicle_idx = select_from_list("Available Vehicles", vehicles);
    if (vehicle_idx < 0) {
        std::cerr << "Invalid vehicle selection.\n";
        return 1;
    }

    const fs::path& track_path = tracks[track_idx];
    const fs::path& vehicle_path = vehicles[vehicle_idx];

    std::cout << "\n----------------------------------------\n";
    std::cout << "Loading track: " << track_path.filename().string() << "\n";

    Track track;
    try {
        track = load_track(track_path.string());
    } catch (const std::exception& e) {
        std::cerr << "Error loading track: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Loading vehicle: " << vehicle_path.filename().string() << "\n";

    VehicleParams vehicle;
    try {
        vehicle = load_vehicle(vehicle_path.string());
    } catch (const std::exception& e) {
        std::cerr << "Error loading vehicle: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nTrack length: " << std::fixed << std::setprecision(0)
              << track.total_length << " m ("
              << std::setprecision(2) << track.total_length / 1000.0 << " km)\n";
    std::cout << "Track segments: " << track.segments.size() << "\n";
    std::cout << "Vehicle mass: " << std::setprecision(1) << vehicle.mass << " kg\n";
    std::cout << "Gears: " << vehicle.drivetrain.gearbox.ratios.size() << "\n\n";

    // Configure simulation
    SimConfig config{};
    config.dt = 0.001;        // 1ms timestep (1000 Hz)
    config.max_time = 300.0;  // 5 minute safety timeout

    std::cout << "Simulation timestep: " << config.dt * 1000.0 << " ms\n";
    std::cout << "Running simulation...\n";

    // Run simulation
    Simulator sim(track, vehicle, config);
    sim.run();

    // Output results
    const double lap_time = sim.lap_time();

    std::cout << "\n========================================\n";
    std::cout << "            LAP COMPLETE\n";
    std::cout << "========================================\n";
    std::cout << "\n  LAP TIME: " << format_lap_time(lap_time) << "\n";
    std::cout << "  (" << std::fixed << std::setprecision(3) << lap_time << " seconds)\n";

    print_telemetry_stats(sim.telemetry());

    std::cout << "\n========================================\n";

    return 0;
}
