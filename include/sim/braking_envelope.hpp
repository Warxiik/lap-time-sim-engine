#pragma once

#include <vector>

#include "core/units.hpp"
#include "models/track.hpp"
#include "models/vehicle.hpp"

/**
 * @file braking_envelope.hpp
 * @brief The fastest speed the car may have anywhere on the lap and still make every corner.
 *
 * A quasi-steady-state lap is the lower of two speed curves along the track:
 *
 *   - forward: accelerating as hard as the engine and the tyres allow;
 *   - backward: the fastest speed from which the car can still brake down to
 *     every corner's limit ahead of it.
 *
 * The simulator integrates the forward curve in time, gear by gear. This is
 * the backward curve, computed once per track and car: the driver model
 * brakes just enough to stay on it, so the car arrives at each corner at the
 * corner's own speed limit without anything clamping its speed.
 */
struct BrakingEnvelope {
    /// Speed standing in for "no limit" (straights; corners the downforce grips faster than they demand), m/s.
    static constexpr double unlimited = 1.0e6;

    meters spacing = 0.0;        // m between nodes; node i is at i * spacing
    meters length = 0.0;         // lap length, m
    std::vector<double> cap;     // m/s: the corner speed limit at each node (lateral grip)
    std::vector<double> limit;   // m/s: the fastest speed at each node that can still brake for every node ahead

    /// The braking envelope at a track position (wrapped into the lap), interpolated between nodes.
    [[nodiscard]] double speed_at(meters position) const;
};

/**
 * @brief Computes the braking envelope for a car on a closed circuit.
 *
 * @param track Track definition (segments, a closed loop)
 * @param vehicle Vehicle parameters
 * @param spacing Node spacing along the track, m (> 0); a node's corner limit also covers any segment starting before the next node
 * @return The envelope, nodes evenly spaced round the lap
 */
BrakingEnvelope compute_braking_envelope(const Track& track, const VehicleParams& vehicle, meters spacing = 0.5);
