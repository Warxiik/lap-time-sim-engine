#pragma once

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

// =============================================================================
// Tyre condition over a stint: temperature, wear and pressure
// =============================================================================
//
// The tyres' condition multiplies TyreParams' friction. It follows the model
// of the game the engine runs in (Understeer's S1 tyre: a tread and a carcass
// node, wear from sliding work, pressure from the carcass temperature), with
// one representative tyre per axle carrying half of the axle's load and forces.
// Off by default: a car's grip is then TyreParams' all stint long.

/// Temperature: two thermal nodes per tyre, and grip from the tread's.
struct TyreThermalParams {
    double optimal_temp = 88.0;                 // °C, tread temperature of peak grip
    double temp_window = 32.0;                  // °C from the optimum where grip is down by grip_loss_at_window
    double grip_loss_at_window = 0.08;          // fraction of grip lost one window from the optimum
    double min_temp_grip = 0.80;                // floor of the temperature grip factor
    double surface_heat_capacity = 900.0;       // J/K, the tread surface
    double carcass_heat_capacity = 2200.0;      // J/K, the carcass and the rest of the tread
    double slip_heat_fraction = 0.8;            // share of the sliding power that heats the tread
    double rolling_heat = 0.015;                // carcass heating per unit of load × speed (W per N·m/s)
    double surface_carcass_conductance = 150.0; // W/K between the two nodes
    double surface_cooling = 4.0;               // W/K from the tread to the air at rest
    double surface_cooling_per_speed = 1.0;     // W/K per m/s
    double road_conductance = 7.0;              // W/K from the tread to the track
    double carcass_cooling = 2.0;               // W/K from the carcass to the air at rest
    double carcass_cooling_per_speed = 0.8;     // W/K per m/s
    double initial_temp = 50.0;                 // °C at the start of the stint (tyre warmers)
    double cold_pressure = 160.0;               // kPa at 20 °C
};

/// Wear: tread lost to sliding work, faster when overheated, with a grip cliff near the end.
struct TyreWearParams {
    double rate = 1.2e-7;           // tread fraction per joule of sliding work
    double overheat_temp = 105.0;   // °C of tread above which wear accelerates
    double overheat_factor = 0.03;  // extra wear per degree above overheat_temp
    double overheat_max = 3.0;      // ceiling of that multiplier
    double grip_loss_linear = 0.05; // grip lost evenly across the tread's life
    double cliff_start = 0.75;      // wear where the cliff begins
    double cliff_loss = 0.15;       // further grip lost from cliff_start to fully worn
};

/// Pressure: the cold pressure scaled with the carcass temperature (ideal gas). Grip falls off either side
/// of an optimal hot pressure, an under-inflated carcass flexes and heats more, and off its optimum a tyre
/// wears faster.
struct TyrePressureParams {
    double optimal = 197.1;           // kPa, hot pressure of peak grip
    double window = 20.0;             // kPa from the optimum where grip is down by grip_loss_at_window
    double grip_loss_at_window = 0.04;
    double min_grip = 0.85;
    double flex_heat_exponent = 1.0;  // rolling heat scales as (optimal / pressure)^this
    double wear_at_window = 0.25;     // extra wear one window from the optimum (growing with its square)
};

/// One axle's tyres.
struct AxleTyres {
    TyreThermalParams thermal;
    TyreWearParams wear;
    TyrePressureParams pressure;
    double peak_slip_angle = 0.12;  // rad, slip angle of the peak lateral force
    double peak_slip_ratio = 0.10;  // slip ratio of the peak longitudinal force
};

/**
 * @struct TyreConditionParams
 * @brief How the tyres heat, wear and hold pressure over a stint.
 *
 * A point mass has no slip, so the work the tyres do sliding is modelled from
 * how much of its grip each axle uses: the tyre's force rises with its
 * normalised slip s as 2s − s² up to the peak at s = 1, so an axle using a
 * share u of its grip slides at s = 1 − √(1 − u) of its peak slip.
 * `sliding_work` scales the result, and is what a calibration fits.
 *
 * The axles share the forces as a car's do: the lateral force by the static
 * weight on each (the yaw balance of a steady corner), braking by the brake
 * bias, driving all on the driven axle; and the load by the static weight, the
 * downforce's balance and the longitudinal load transfer.
 */
struct TyreConditionParams {
    bool enabled = false;
    AxleTyres front;
    AxleTyres rear;
    double weight_front = 0.5;              // share of the static weight on the front axle
    double aero_front = 0.5;                // share of the downforce on the front axle
    double brake_front = 0.6;               // share of the braking force at the front
    bool front_driven = false;
    double cg_height_over_wheelbase = 0.2;  // longitudinal load transfer: ΔN = m·a·h/L
    double sliding_work = 1.0;              // scale on the modelled sliding work
    double ambient_temp = 20.0;             // °C
    double track_temp = 25.0;               // °C
};

/// One axle's tyres now.
struct TyreState {
    double tread_temp = 20.0;   // °C
    double carcass_temp = 20.0; // °C
    double wear = 0.0;          // 0 new .. 1 worn
    double pressure = 0.0;      // kPa, hot
    double grip = 1.0;          // multiplier of TyreParams' friction: temperature × wear × pressure
};
