#pragma once

#include "models/car_state.hpp"
#include "models/control.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"
#include "core/units.hpp"

namespace step {

        /// Advances `state` by `dt` in `segment` (the one the car is in) with the driver's inputs.
        void advance(
            CarState& state,
            const VehicleParams& vehicle,
            const TrackSegment& segment,
            const ControlInput& control,
            seconds dt
        );

} // namespace step
