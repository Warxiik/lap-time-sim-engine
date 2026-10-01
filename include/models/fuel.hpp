#pragma once

/**
 * @struct FuelParams
 * @brief The fuel aboard and how fast the engine burns it.
 *
 * The fuel's mass is on top of VehicleParams::mass, and falls as the engine
 * burns it: brake-specific consumption on the crank's work, plus an idle flow.
 * With no fuel (the default) the car's mass is VehicleParams::mass all stint.
 */
struct FuelParams {
    double mass = 0.0;       // kg aboard at the start
    double bsfc = 0.0;       // g of fuel per kWh of crank work (0: burns nothing)
    double idle_flow = 0.0;  // kg/s while the engine turns
};
