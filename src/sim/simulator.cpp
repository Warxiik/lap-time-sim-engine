#include "sim/simulator.hpp"
#include "sim/sim_config.hpp"
#include "core/units.hpp"

Simulator::Simulator(Track track, VehicleParams vehicle, SimConfig config)
    : track_(std::move(track)),
      vehicle_(std::move(vehicle)),
      config_(config)
{
    state_ = {};
    state_.s = 0.0;
    state_.v = 0.0;
}

void Simulator::run() {
    const seconds dt = config_.dt;

    while (state_.s < track_.total_length) {

        const ControlInput control = driver_.compute_control(state_, track_);

        step::advance(
            state_,
            vehicle_,
            track_,
            control,
            dt
        );

        telemetry_.record(state_, control);
    }
}

const Telemetry& Simulator::telemetry() const {
    return telemetry_;
}

seconds Simulator::lap_time() const {
    return telemetry_.time.back();
}
