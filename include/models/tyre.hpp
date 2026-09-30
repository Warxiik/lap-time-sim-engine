#pragma once

struct TyreState {
    double temperature; // normalized [0..1]
    double wear;        // normalized [0..1]
};

/**
 * @struct TyreParams
 * @brief Friction the tyres give on a surface of grip 1.0.
 *
 * The friction coefficient in each direction is the tyre's value times the
 * track segment's grip, so a segment of grip 0.7 (wet) scales both. Tyres
 * usually grip a little more under braking and traction than in cornering,
 * which makes the friction envelope an ellipse rather than a circle.
 *
 * Both default to 1.0: the track's grip alone, as before tyres were modelled.
 */
struct TyreParams {
    double base_grip = 1.0;          // lateral friction coefficient μ at optimal conditions
    double longitudinal_grip = 1.0;  // friction coefficient μ for driving and braking
};
