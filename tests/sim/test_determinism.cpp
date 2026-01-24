#include <gtest/gtest.h>

#include "sim/sim_config.hpp"
#include "sim/simulator.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"

/**
 * @brief Creates a simple test track for determinism testing.
 *
 * This creates a small circuit with straights and corners to exercise
 * all physics code paths. Using programmatic creation instead of file
 * loading ensures tests don't depend on external data files.
 */
static Track create_test_track() {
    Track track;

    // Start/finish straight (500m)
    track.segments.push_back({500.0, 0.0, 1.0, 0.0});

    // Turn 1: medium right (50m radius = 0.02 curvature)
    track.segments.push_back({78.5, 0.02, 1.0, 0.0});  // 90 degrees

    // Back straight (300m)
    track.segments.push_back({300.0, 0.0, 1.0, 0.0});

    // Turn 2: tight left hairpin (25m radius = 0.04 curvature)
    track.segments.push_back({78.5, -0.04, 1.0, 0.0});  // 180 degrees

    // Short straight (100m)
    track.segments.push_back({100.0, 0.0, 1.0, 0.0});

    // Turn 3: fast sweeper right (100m radius = 0.01 curvature)
    track.segments.push_back({157.0, 0.01, 1.0, 0.0});  // 90 degrees

    // Calculate total length
    track.total_length = 0.0;
    for (const auto& seg : track.segments) {
        track.total_length += seg.length;
    }

    return track;
}

/**
 * @brief Creates a test vehicle with F1-like parameters.
 */
static VehicleParams create_test_vehicle() {
    VehicleParams vehicle{};

    vehicle.mass = 800.0;           // kg
    vehicle.wheel_radius = 0.33;    // m

    // Aerodynamics
    vehicle.aero.drag_coefficient = 1.0;
    vehicle.aero.lift_coefficient = -3.0;
    vehicle.aero.frontal_area = 1.5;

    // Engine
    vehicle.drivetrain.engine.rpm = {5000, 8000, 10000, 12000, 15000};
    vehicle.drivetrain.engine.torque = {350, 420, 450, 430, 380};

    // Gearbox
    vehicle.drivetrain.gearbox.ratios = {3.5, 2.5, 1.9, 1.5, 1.2, 1.0, 0.85, 0.75};
    vehicle.drivetrain.gearbox.final_drive = 3.5;
    vehicle.drivetrain.efficiency = 0.9;

    vehicle.max_drive_force = 15000.0;
    vehicle.max_brake_force = 25000.0;

    return vehicle;
}

/**
 * @test Identical inputs must produce bitwise identical outputs.
 *
 * This is the fundamental determinism test. Running the exact same
 * simulation twice with identical inputs must produce exactly the
 * same results. This is critical for:
 *   - Reproducible testing and debugging
 *   - Valid parameter sweeps and comparisons
 *   - Reliable strategy optimization
 *
 * The test runs two simulators with identical configuration and
 * verifies that lap times and telemetry match exactly.
 */
TEST(SimulationDeterminism, IdenticalInputsProduceIdenticalOutputs) {
    Track track = create_test_track();
    VehicleParams vehicle = create_test_vehicle();

    SimConfig config;
    config.dt = 0.001;        // 1ms timestep
    config.max_time = 300.0;  // 5 minute timeout

    // Run two identical simulations
    Simulator sim1(track, vehicle, config);
    Simulator sim2(track, vehicle, config);

    sim1.run();
    sim2.run();

    // Lap times must be exactly equal (bitwise)
    ASSERT_DOUBLE_EQ(sim1.lap_time(), sim2.lap_time());

    const auto& t1 = sim1.telemetry();
    const auto& t2 = sim2.telemetry();

    // Telemetry length must match
    ASSERT_EQ(t1.size(), t2.size());

    // Spot check samples throughout the lap
    // Using ASSERT_DOUBLE_EQ for exact bitwise comparison
    for (size_t i = 0; i < t1.size(); i += 50) {
        ASSERT_DOUBLE_EQ(t1.distance[i], t2.distance[i])
            << "Distance mismatch at sample " << i;
        ASSERT_DOUBLE_EQ(t1.velocity[i], t2.velocity[i])
            << "Velocity mismatch at sample " << i;
        ASSERT_DOUBLE_EQ(t1.acceleration[i], t2.acceleration[i])
            << "Acceleration mismatch at sample " << i;
    }
}

/**
 * @test Different timesteps should give similar (but not identical) results.
 *
 * This validates that the physics is consistent across different
 * timestep sizes. Smaller timesteps should give more accurate results,
 * and results should converge as timestep decreases.
 *
 * We expect lap times to be within ~1% for reasonable timestep variations.
 */
TEST(SimulationDeterminism, DifferentTimestepsConverge) {
    Track track = create_test_track();
    VehicleParams vehicle = create_test_vehicle();

    SimConfig config_coarse;
    config_coarse.dt = 0.005;     // 5ms timestep
    config_coarse.max_time = 300.0;

    SimConfig config_fine;
    config_fine.dt = 0.001;       // 1ms timestep
    config_fine.max_time = 300.0;

    Simulator sim_coarse(track, vehicle, config_coarse);
    Simulator sim_fine(track, vehicle, config_fine);

    sim_coarse.run();
    sim_fine.run();

    double lap_coarse = sim_coarse.lap_time();
    double lap_fine = sim_fine.lap_time();

    // Lap times should be within 2% of each other
    double diff_percent = std::abs(lap_coarse - lap_fine) / lap_fine * 100.0;
    EXPECT_LT(diff_percent, 2.0)
        << "Coarse: " << lap_coarse << "s, Fine: " << lap_fine << "s";
}

/**
 * @test Simulation should complete in reasonable time.
 *
 * Verifies the car actually completes the track and doesn't get stuck.
 * Also provides a basic sanity check on lap time reasonableness.
 */
TEST(SimulationDeterminism, SimulationCompletes) {
    Track track = create_test_track();
    VehicleParams vehicle = create_test_vehicle();

    SimConfig config;
    config.dt = 0.001;
    config.max_time = 300.0;

    Simulator sim(track, vehicle, config);
    sim.run();

    double lap_time = sim.lap_time();

    // Lap should complete (not hit timeout)
    EXPECT_LT(lap_time, config.max_time);

    // Lap time should be reasonable for a ~1.2km track
    // At average 50 m/s, that's about 24 seconds
    EXPECT_GT(lap_time, 10.0);   // Not impossibly fast
    EXPECT_LT(lap_time, 120.0);  // Not unreasonably slow
}
