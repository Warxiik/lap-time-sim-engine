#include <gtest/gtest.h>

#include "sim/sim_config.hpp"
#include "sim/simulator.hpp"
#include "io/track_loader.hpp"
#include "io/vehicle_loader.hpp"

TEST(SimulationDeterminism, IdenticalInputsProduceIdenticalOutputs) {
    Track track = load_track("data/tracks/monza.csv");
    VehicleParams vehicle = load_vehicle("data/vehicles/sports_car.json");

    SimConfig config;
    config.dt = 0.002;

    Simulator sim1(track, vehicle, config);
    Simulator sim2(track, vehicle, config);

    sim1.run();
    sim2.run();

    ASSERT_DOUBLE_EQ(sim1.lap_time(), sim2.lap_time());

    const auto& t1 = sim1.telemetry();
    const auto& t2 = sim2.telemetry();

    ASSERT_EQ(t1.s.size(), t2.s.size());

    // Spot check samples
    for (size_t i = 0; i < t1.s.size(); i+=50) {
        ASSERT_DOUBLE_EQ(t1.s[i], t2.s[i]);
        ASSERT_DOUBLE_EQ(t1.v[i], t2.v[i]);
    }
}