#pragma once

#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/telemetry.hpp"
#include "sim/driver_model.hpp"
#include "sim/braking_envelope.hpp"
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

    /// The timed lap, from the line to the line (interpolated within the
    /// step that crosses it). Without a finished lap: the time driven.
    seconds lap_time() const;

    /// True once the timed lap reached the line before max_time.
    bool completed() const;

    /// The braking envelope the driver follows.
    const BrakingEnvelope& braking_envelope() const;

private:
    /// One step: driver inputs, gear, physics. Returns the inputs used.
    ControlInput step_once(seconds dt);

    Track track_;
    VehicleParams vehicle_;
    SimConfig config_;
    BrakingEnvelope envelope_;

    CarState state_;
    Telemetry telemetry_;
    DriverModel driver_;

    seconds lap_time_ = 0.0;
    bool completed_ = false;
};
