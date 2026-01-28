/**
 * @file benchmark.cpp
 * @brief Performance benchmarking tool for the lap time simulator.
 *
 * Measures simulation performance across multiple runs and configurations.
 * Reports throughput (laps/second) and verifies determinism.
 *
 * Usage: benchmark [num_iterations]
 *   Default: 100 iterations
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>

#include "sim/simulator.hpp"
#include "sim/sim_config.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"

using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double, std::milli>;

// ============================================================================
// Test Data Creation
// ============================================================================

/**
 * @brief Creates a benchmark track (~5km with varied corners).
 */
Track create_benchmark_track() {
    Track track;

    // Mix of straights and corners to exercise all physics
    track.segments.push_back({600.0, 0.0, 1.0, 0.0});      // Straight
    track.segments.push_back({120.0, 0.015, 1.0, 0.0});    // Medium right
    track.segments.push_back({300.0, 0.0, 1.0, 0.0});      // Straight
    track.segments.push_back({80.0, -0.025, 1.0, 0.0});    // Tight left
    track.segments.push_back({500.0, 0.0, 1.0, 0.0});      // Long straight
    track.segments.push_back({200.0, 0.008, 1.0, 0.0});    // Fast sweeper
    track.segments.push_back({150.0, 0.0, 1.0, 0.0});      // Short straight
    track.segments.push_back({90.0, 0.035, 1.0, 0.0});     // Hairpin
    track.segments.push_back({400.0, 0.0, 1.0, 0.0});      // Straight
    track.segments.push_back({180.0, -0.012, 1.0, 0.0});   // Medium left
    track.segments.push_back({350.0, 0.0, 1.0, 0.0});      // Straight
    track.segments.push_back({100.0, 0.02, 1.0, 0.0});     // Chicane entry
    track.segments.push_back({100.0, -0.02, 1.0, 0.0});    // Chicane exit
    track.segments.push_back({550.0, 0.0, 1.0, 0.0});      // Pit straight

    track.total_length = 0.0;
    for (const auto& seg : track.segments) {
        track.total_length += seg.length;
    }

    return track;
}

/**
 * @brief Creates an F1-spec benchmark vehicle.
 */
VehicleParams create_benchmark_vehicle() {
    VehicleParams vehicle{};

    vehicle.mass = 798.0;
    vehicle.wheel_radius = 0.33;

    vehicle.aero.drag_coefficient = 0.9;
    vehicle.aero.lift_coefficient = -3.5;
    vehicle.aero.frontal_area = 1.5;

    vehicle.drivetrain.engine.rpm = {5000, 7000, 9000, 11000, 13000, 15000};
    vehicle.drivetrain.engine.torque = {420, 500, 540, 530, 490, 440};

    vehicle.drivetrain.gearbox.ratios = {3.2, 2.5, 2.0, 1.6, 1.3, 1.1, 1.0, 0.9};
    vehicle.drivetrain.gearbox.final_drive = 3.4;
    vehicle.drivetrain.efficiency = 0.95;

    vehicle.max_drive_force = 18000.0;
    vehicle.max_brake_force = 35000.0;

    return vehicle;
}

// ============================================================================
// Statistics
// ============================================================================

struct BenchmarkStats {
    double mean_ms;
    double std_dev_ms;
    double min_ms;
    double max_ms;
    double laps_per_second;
    size_t iterations;
};

BenchmarkStats compute_stats(const std::vector<double>& times_ms) {
    BenchmarkStats stats{};
    stats.iterations = times_ms.size();

    if (times_ms.empty()) {
        return stats;
    }

    // Mean
    stats.mean_ms = std::accumulate(times_ms.begin(), times_ms.end(), 0.0)
                    / static_cast<double>(times_ms.size());

    // Min/Max
    stats.min_ms = *std::min_element(times_ms.begin(), times_ms.end());
    stats.max_ms = *std::max_element(times_ms.begin(), times_ms.end());

    // Standard deviation
    double variance = 0.0;
    for (double t : times_ms) {
        variance += (t - stats.mean_ms) * (t - stats.mean_ms);
    }
    variance /= static_cast<double>(times_ms.size());
    stats.std_dev_ms = std::sqrt(variance);

    // Throughput
    stats.laps_per_second = 1000.0 / stats.mean_ms;

    return stats;
}

// ============================================================================
// Benchmark Tests
// ============================================================================

/**
 * @brief Runs the main performance benchmark.
 */
BenchmarkStats run_performance_benchmark(int iterations) {
    const Track track = create_benchmark_track();
    const VehicleParams vehicle = create_benchmark_vehicle();

    SimConfig config{};
    config.dt = 0.001;
    config.max_time = 300.0;

    std::vector<double> times_ms;
    times_ms.reserve(iterations);

    // Warmup run (not measured)
    {
        Simulator warmup(track, vehicle, config);
        warmup.run();
    }

    // Measured runs
    for (int i = 0; i < iterations; ++i) {
        Simulator sim(track, vehicle, config);

        auto start = Clock::now();
        sim.run();
        auto end = Clock::now();

        Duration elapsed = end - start;
        times_ms.push_back(elapsed.count());
    }

    return compute_stats(times_ms);
}

/**
 * @brief Verifies simulation determinism.
 */
bool verify_determinism(int iterations) {
    const Track track = create_benchmark_track();
    const VehicleParams vehicle = create_benchmark_vehicle();

    SimConfig config{};
    config.dt = 0.001;
    config.max_time = 300.0;

    // Get reference result
    Simulator reference(track, vehicle, config);
    reference.run();
    const double reference_lap_time = reference.lap_time();

    // Compare subsequent runs
    for (int i = 0; i < iterations; ++i) {
        Simulator sim(track, vehicle, config);
        sim.run();

        if (sim.lap_time() != reference_lap_time) {
            std::cerr << "Determinism FAILED at iteration " << i << "\n";
            std::cerr << "  Reference: " << std::setprecision(15) << reference_lap_time << "\n";
            std::cerr << "  Got:       " << std::setprecision(15) << sim.lap_time() << "\n";
            return false;
        }
    }

    return true;
}

/**
 * @brief Benchmarks different timestep sizes.
 */
void benchmark_timesteps() {
    const Track track = create_benchmark_track();
    const VehicleParams vehicle = create_benchmark_vehicle();

    const std::vector timesteps = {0.01, 0.005, 0.002, 0.001, 0.0005};

    std::cout << "\n=== Timestep Comparison ===\n";
    std::cout << std::setw(12) << "Timestep"
              << std::setw(15) << "Lap Time"
              << std::setw(15) << "Exec Time"
              << std::setw(12) << "Steps"
              << "\n";
    std::cout << std::string(54, '-') << "\n";

    double reference_lap = 0.0;

    for (const double dt : timesteps) {
        SimConfig config{};
        config.dt = dt;
        config.max_time = 300.0;

        Simulator sim(track, vehicle, config);

        auto start = Clock::now();
        sim.run();
        auto end = Clock::now();

        Duration elapsed = end - start;
        const double lap_time = sim.lap_time();

        if (reference_lap == 0.0) {
            reference_lap = lap_time;
        }

        std::cout << std::fixed << std::setprecision(4)
                  << std::setw(10) << (dt * 1000) << " ms"
                  << std::setw(12) << lap_time << " s"
                  << std::setw(12) << elapsed.count() << " ms"
                  << std::setw(12) << sim.telemetry().size()
                  << "\n";
    }
}

/**
 * @brief Prints system/build information.
 */
void print_system_info() {
    std::cout << "=== System Information ===\n";
    std::cout << "Compiler: ";
#if defined(__clang__)
    std::cout << "Clang " << __clang_major__ << "." << __clang_minor__;
#elif defined(__GNUC__)
    std::cout << "GCC " << __GNUC__ << "." << __GNUC_MINOR__;
#elif defined(_MSC_VER)
    std::cout << "MSVC " << _MSC_VER;
#else
    std::cout << "Unknown";
#endif
    std::cout << "\n";

    std::cout << "Build: ";
#ifdef NDEBUG
    std::cout << "Release";
#else
    std::cout << "Debug";
#endif
    std::cout << "\n";

    std::cout << "sizeof(double): " << sizeof(double) << " bytes\n";
    std::cout << "\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    int iterations = 100;

    if (argc > 1) {
        iterations = std::atoi(argv[1]);
        if (iterations <= 0) {
            std::cerr << "Invalid iteration count. Using default (100).\n";
            iterations = 100;
        }
    }

    std::cout << "========================================\n";
    std::cout << "    LAP TIME SIMULATOR BENCHMARK\n";
    std::cout << "========================================\n\n";

    print_system_info();

    // Benchmark track info
    auto [segments, total_length] = create_benchmark_track();
    std::cout << "=== Benchmark Configuration ===\n";
    std::cout << "Track length: " << total_length << " m\n";
    std::cout << "Track segments: " << segments.size() << "\n";
    std::cout << "Timestep: 1.0 ms\n";
    std::cout << "Iterations: " << iterations << "\n\n";

    // Determinism check
    std::cout << "=== Determinism Check ===\n";
    std::cout << "Verifying determinism over " << iterations << " runs... ";
    std::cout.flush();

    if (verify_determinism(iterations)) {
        std::cout << "PASSED\n\n";
    } else {
        std::cout << "FAILED\n";
        return 1;
    }

    // Performance benchmark
    std::cout << "=== Performance Benchmark ===\n";
    std::cout << "Running " << iterations << " iterations...\n";

    const BenchmarkStats stats = run_performance_benchmark(iterations);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nResults:\n";
    std::cout << "  Mean:      " << stats.mean_ms << " ms/lap\n";
    std::cout << "  Std Dev:   " << stats.std_dev_ms << " ms\n";
    std::cout << "  Min:       " << stats.min_ms << " ms\n";
    std::cout << "  Max:       " << stats.max_ms << " ms\n";
    std::cout << "  Throughput: " << std::setprecision(1) << stats.laps_per_second << " laps/second\n";

    // Timestep comparison
    benchmark_timesteps();

    std::cout << "\n========================================\n";
    std::cout << "          BENCHMARK COMPLETE\n";
    std::cout << "========================================\n";

    return 0;
}
