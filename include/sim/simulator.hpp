#pragma once

#include <vector>

#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "models/car_state.hpp"
#include "models/telemetry.hpp"
#include "sim/driver_model.hpp"
#include "sim/braking_envelope.hpp"
#include "sim/segment_index.hpp"
#include "sim/gear_map.hpp"
#include "sim/step.hpp"
#include "sim/sim_config.hpp"
#include "core/units.hpp"

using seconds = double;

/// One timed lap of a stint.
struct LapRecord {
    seconds time = 0.0;          // line to line
    double fuel_at_start = 0.0;  // kg aboard as the lap began
    double fuel_at_end = 0.0;    // kg aboard as it ended
    TyreState front;             // each axle's tyres as the lap ended
    TyreState rear;
};

class Simulator {
public:
    Simulator(
        Track track,
        VehicleParams vehicle,
        SimConfig config
        );
    void run();

    /// Every step of the timed laps. Distance runs on from lap to lap (lap k from k × the lap's length).
    const Telemetry& telemetry() const;

    /// The first timed lap, from the line to the line (interpolated within the
    /// step that crosses it). Without a finished lap: the time driven.
    seconds lap_time() const;

    /// True once every timed lap (SimConfig::laps) reached the line before max_time.
    bool completed() const;

    /// The timed laps finished, in order.
    const std::vector<LapRecord>& laps() const;

    /// The braking envelope the driver follows (the last one planned).
    const BrakingEnvelope& braking_envelope() const;

    /// The car as it is now: VehicleParams with the fuel left in its mass and the tyres' condition in its grip.
    const VehicleParams& vehicle_now() const;

    /// The car's state now (its fuel and tyres too).
    const CarState& state() const;

private:
    /// One step: gear, driver inputs, physics, and what the step cost in fuel and tyre. Returns the inputs used.
    ControlInput step_once(seconds dt);

    /// Fuel or tyre condition modelled: the car changes as it drives.
    bool consumables() const;
    /// The fuel after a step that started in `before`.
    void burn(const CarState& before, const ControlInput& control, seconds dt);
    /// The tyres over `dt` from `before`, putting `tyre_force` through the road.
    void wear_tyres(const CarState& before, const TrackSegment& segment, double tyre_force, seconds dt);
    /// current_ from the car, the fuel left and the tyres' grip.
    void update_vehicle();
    /// A braking envelope for the car as it is now.
    void replan();

    Track track_;
    VehicleParams vehicle_;
    SimConfig config_;
    VehicleParams current_;
    BrakingEnvelope envelope_;
    SegmentIndex index_;
    GearMap gears_;

    CarState state_;
    Telemetry telemetry_;
    DriverModel driver_;

    double start_mass_ = 0.0;     // kg, the car with its starting fuel
    int tyre_steps_ = 1;          // steps between updates of the tyres' condition (every 10 ms)
    int tyre_countdown_ = 0;      // steps until the next one
    double envelope_grip_ = 1.0;  // the tyres' grip the envelope was planned with
    double envelope_scale_ = 1.0; // √(grip now / envelope_grip_): the driver's speeds between plans
    std::vector<LapRecord> laps_;
    seconds lap_time_ = 0.0;
    bool completed_ = false;
};
