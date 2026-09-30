#include "sim/braking_envelope.hpp"
#include "physics/lateral.hpp"
#include "physics/vehicle_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

/**
 * @brief Interpolates the braking envelope at a track position.
 *
 * Positions outside [0, length) wrap round the lap, so a driver looking past
 * the finish line sees the first corners of the next lap.
 */
double BrakingEnvelope::speed_at(meters position) const {
    const size_t n = limit.size();
    if (n == 0) {
        return unlimited;
    }
    double s = std::fmod(position, length);
    if (s < 0.0) {
        s += length;
    }
    const double u = s / spacing;
    size_t i = static_cast<size_t>(u);
    if (i >= n) {
        i = n - 1;
    }
    const double t = u - static_cast<double>(i);
    return limit[i] + t * (limit[(i + 1) % n] - limit[i]);
}

namespace {

/**
 * @brief Deceleration the car can reach at a speed on a segment (m/s², >= 0).
 *
 * The brakes as far as the tyres transmit them (the friction ellipse leaves
 * less in a corner), plus aerodynamic drag, which helps braking.
 */
double braking_deceleration(double velocity, const VehicleParams& vehicle, const TrackSegment& segment) {
    CarState state{};
    state.v = velocity;
    const physics::LongitudinalForces forces = physics::longitudinal_forces(state, vehicle, segment);
    return (std::min(forces.brakes, forces.grip) + forces.drag) / vehicle.mass;
}

} // namespace

/**
 * @brief Computes the braking envelope on a closed circuit.
 *
 * === Nodes and corner limits ===
 *
 * The lap is cut into evenly spaced nodes. Each node's cap is the steady
 * corner speed (physics::lateral::max_speed) of its segment, and of every
 * segment that starts before the next node, so a short or sudden corner
 * between two nodes is never missed.
 *
 * === The backward pass ===
 *
 * Braking over one node spacing h from speed v_next reaches back to
 *
 *   v = √(v_next² + 2 · a_brake · h)
 *
 * and each node's limit is the lower of that and its own cap. The pass
 * starts at the slowest cap of the lap, where the limit is the cap itself
 * whatever comes after it, and walks backwards round the whole lap once.
 *
 * The deceleration is the least of two estimates, at the speed braked from
 * and at the speed braked to, on every segment the interval overlaps, so
 * the envelope never asks for more braking than the car has anywhere in
 * between. That matters most where a corner tightens: braking and cornering
 * share the grip, and past the corner limit there is none left to brake
 * with, so a car that fell even slightly behind the envelope there would
 * not catch up.
 */
BrakingEnvelope compute_braking_envelope(const Track& track, const VehicleParams& vehicle, meters spacing) {
    if (!(spacing > 0.0)) {
        throw std::invalid_argument("braking envelope: spacing must be positive");
    }
    if (track.segments.empty() || !(track.total_length > 0.0)) {
        throw std::invalid_argument("braking envelope: empty track");
    }

    BrakingEnvelope envelope;
    const size_t n = std::max<size_t>(3, static_cast<size_t>(std::ceil(track.total_length / spacing)));
    envelope.length = track.total_length;
    envelope.spacing = track.total_length / static_cast<double>(n);
    envelope.cap.assign(n, BrakingEnvelope::unlimited);
    envelope.limit.assign(n, BrakingEnvelope::unlimited);

    // The segment at each node, and each segment's corner limit applied to the node interval it starts in.
    std::vector<size_t> node_segment(n, 0);
    {
        size_t seg = 0;
        double seg_end = track.segments[0].length;
        for (size_t i = 0; i < n; ++i) {
            const double s = static_cast<double>(i) * envelope.spacing;
            while (s >= seg_end && seg + 1 < track.segments.size()) {
                ++seg;
                seg_end += track.segments[seg].length;
            }
            node_segment[i] = seg;
        }
    }
    std::vector<double> segment_cap(track.segments.size());
    for (size_t j = 0; j < track.segments.size(); ++j) {
        segment_cap[j] = std::min(physics::lateral::max_speed(track.segments[j], vehicle), BrakingEnvelope::unlimited);
    }
    for (size_t i = 0; i < n; ++i) {
        envelope.cap[i] = segment_cap[node_segment[i]];
    }
    double start = 0.0;
    for (size_t j = 0; j < track.segments.size(); ++j) {
        const size_t i = std::min(n - 1, static_cast<size_t>(start / envelope.spacing));
        envelope.cap[i] = std::min(envelope.cap[i], segment_cap[j]);
        start += track.segments[j].length;
    }

    // Backward from the slowest node, once round the lap.
    const size_t slowest = static_cast<size_t>(std::min_element(envelope.cap.begin(), envelope.cap.end()) - envelope.cap.begin());
    envelope.limit[slowest] = envelope.cap[slowest];
    const size_t segments = track.segments.size();
    for (size_t k = 1; k < n; ++k) {
        const size_t i = (slowest + n - k) % n;
        const size_t next = (i + 1) % n;
        const double v_next = envelope.limit[next];
        double v = BrakingEnvelope::unlimited;
        if (v_next < BrakingEnvelope::unlimited) {
            // The least braking any segment in the interval gives: the car may be in either end's (or one
            // between) while it brakes across it, and a tighter corner leaves less of the grip to brake with.
            const size_t first = node_segment[i];
            const size_t last = next == 0 ? segments - 1 : node_segment[next];
            const auto interval_deceleration = [&](double velocity) {
                double a = braking_deceleration(velocity, vehicle, track.segments[first]);
                for (size_t j = first; j != last; j = (j + 1) % segments) {
                    a = std::min(a, braking_deceleration(velocity, vehicle, track.segments[(j + 1) % segments]));
                }
                return a;
            };
            const double h = envelope.spacing;
            const double a_from = interval_deceleration(v_next);
            const double v_guess = std::sqrt(v_next * v_next + 2.0 * a_from * h);
            const double a = std::min(a_from, interval_deceleration(v_guess));
            v = std::sqrt(v_next * v_next + 2.0 * a * h);
        }
        envelope.limit[i] = std::min(envelope.cap[i], v);
    }
    return envelope;
}
