#pragma once

#include "models/car_state.hpp"
#include "models/control.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "core/units.hpp"

namespace step {

        void advance(
            CarState& state,
            const VehicleParams& vehicle,
            const Track& track,
            const ControlInput& control,
            seconds dt
        );

} // namespace step