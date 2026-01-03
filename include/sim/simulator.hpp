#pragma once

#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/telemetry.hpp"
#include "sim/driver_model.hpp"
#include "sim/step.hpp"
#include "sim/sim_config.hpp"
#include "core/units.hpp"

using seconds = double;

class Simulator {
public:
    Simulator(
        Track track,
        VehicleParams vehicle,
        SimConfig config
        );
    void run();
    const Telemetry& telemetry() const;
    seconds lap_time() const;

private:
    Track track_;
    VehicleParams vehicle_;
    SimConfig config_;

    CarState state_;
    Telemetry telemetry_;

    DriverModel driver;

};