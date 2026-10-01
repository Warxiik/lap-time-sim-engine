# Physics Model Documentation

This document describes the physics equations and models used in the Lap Time Simulation Engine.

## Overview

The simulator uses a **point-mass model** with the following characteristics:
- Vehicle treated as a single point with mass
- Forces applied at the center of mass
- No suspension geometry or weight transfer
- Curvilinear (distance-based) coordinate system along the track centerline

This approach prioritizes computational efficiency and determinism over full vehicle dynamics fidelity.

---

## Coordinate System

The simulation uses a **curvilinear coordinate system**:
- **s**: Distance along the track centerline (meters)
- **v**: Velocity tangent to the track (m/s)
- **a**: Acceleration tangent to the track (m/s²)

This simplifies the problem from 2D/3D Cartesian coordinates to 1D motion along the track.

---

## Aerodynamics

### Drag Force

Aerodynamic drag opposes the vehicle's motion through air:

```
F_drag = 0.5 × ρ × Cd × A × v²
```

Where:
- `ρ` = Air density (1.225 kg/m³ at sea level)
- `Cd` = Drag coefficient (dimensionless)
- `A` = Frontal area (m²)
- `v` = Velocity (m/s)

**Typical values:**
| Vehicle Type | Cd | A (m²) |
|--------------|-----|--------|
| F1 Car | 0.9-1.1 | 1.5 |
| GT3 Car | 0.5-0.6 | 2.0 |
| Road Car | 0.3-0.4 | 2.0 |

### Downforce

Aerodynamic downforce pushes the vehicle into the ground:

```
F_downforce = 0.5 × ρ × |Cl| × A × v²
```

Where:
- `Cl` = Lift coefficient (negative for downforce)

**Sign convention:** Cl is negative for downforce (inverted wing). The function returns positive downforce magnitude.

**Why downforce matters:**
- Increases normal force on tires without adding mass
- More normal force = more friction = higher cornering speeds
- F1 cars generate 3-4× their weight in downforce at high speed

---

## Longitudinal Dynamics

### Drive Force

The force delivered to the wheels from the powertrain:

```
F_drive = (T_engine × gear_ratio × final_drive × η) / r_wheel
```

Where:
- `T_engine` = Engine torque at current RPM (Nm)
- `gear_ratio` = Current gear ratio
- `final_drive` = Differential ratio
- `η` = Drivetrain efficiency (typically 0.85-0.95)
- `r_wheel` = Wheel radius (m)

Engine torque is interpolated from a torque curve (RPM vs torque table).

`max_drive_force` caps it: what the driven wheels transmit on a straight. The driven tyres corner too, so in a corner the cap shrinks by the same friction ellipse as the tyres' grip (see *Tyre Limit*):

```
F_drive ≤ max_drive_force × ellipse
```

### Gear Selection

At every step the car is in the gear with the most drive force at its speed: of the gears that keep the engine at or under its torque curve's last RPM, the one with the largest torque × ratio (top gear if none does). The gear depends on the speed alone, so the time integration is the forward curve of the best gear at every speed.

Shifting at RPM thresholds (up at 90 %, down at 40 % of the last RPM, as before) made the gear depend on the gear before it. A car leaving a corner just above the downshift RPM stayed a gear too high down the whole straight after it, and a few kilograms more or less decided which: the lap time jumped by tenths between two masses a kilogram apart.

The simulator works the shift speeds out once per car (`GearMap`): it samples the speeds every 0.05 m/s up to past top gear's last RPM, and bisects each change of gear down to the exact speed. Each step then only looks its speed up among them. The gear is selected first in a step, with the engine speed it gives, so the driver's inputs and the physics use the same forces.

### Brake Force

The brakes' maximum force is a constant parameter representing the combined limit of:
- Brake disc/caliper capacity
- Brake-by-wire system limits

```
F_brake = brake_pedal × max_brake_force
```

### Tyre Limit

Neither the engine nor the brakes can put more through the tyres than they grip:

```
F_tyre = clamp(F_drive − F_brake, −F_long_max, +F_long_max)
F_long_max = μ_long × m × a_normal × ellipse
```

with `μ_long` the tyres' longitudinal friction, `a_normal` the load per unit mass (see the tyre model) and `ellipse` the share of the longitudinal grip the corner leaves.

### Net Longitudinal Force

```
F_net = F_tyre - F_drag
```

The acceleration is then:
```
a = F_net / m
```

---

## Lateral Dynamics

### Maximum Cornering Speed

The maximum speed through a corner is where the lateral grip, with the downforce at that same speed, just holds the corner. On a flat road:

```
v² · κ = μ · (g + q · v²),    q = ρ · |Cl| · A / (2m)

v_max = √(μ · g / (κ − μ · q))
```

On a road banked by θ into the turn (the segment's `camber`), part of the weight helps the car round and part of the cornering load presses it into the road:

```
v²·κ·cos θ − g·sin θ = μ · (g·cos θ + v²·κ·sin θ + q·v²)

v_max = √(g · (μ·cos θ + sin θ) / (κ · (cos θ − μ·sin θ) − μ · q))
```

Where:
- `μ` = the tyres' lateral friction (`tyre.base_grip`) times the segment's grip
- `κ` = Track curvature (1/radius, in 1/m)
- `θ` = Banking into the turn (rad)

If the denominator is not positive, the downforce grips faster than the corner demands and the corner sets no limit: power and drag do.

The corner speed is the corner's own. It does not depend on the speed the car arrives with.

### Lateral Acceleration Limit

The maximum lateral acceleration depends on tire grip and normal force:

```
a_lat_max = μ × F_normal / m
          = μ × (m×g + F_downforce) / m
          = μ × g + μ × F_downforce / m
```

Where:
- `μ` = Tire friction coefficient (grip)
- `F_normal` = Normal force (weight + downforce)
- `m` = Vehicle mass
- `g` = Gravitational acceleration (9.81 m/s²)

**Key insight:** Downforce increases grip without increasing the mass that must be accelerated. This is why high-downforce cars corner faster.

---

## Tyre Model (Friction Ellipse)

### Concept

Tyres have a limited total grip budget shared between:
- Lateral force (cornering)
- Longitudinal force (acceleration/braking)

The combinations a tyre can hold fill an ellipse: its axes are the lateral and the longitudinal friction, which usually differ a little.

### Friction

```
μ_lat  = tyre.base_grip         × segment.grip
μ_long = tyre.longitudinal_grip × segment.grip
```

Both tyre values are optional in the vehicle JSON (`"tyre": { "base_grip": 1.6, "longitudinal_grip": 1.7 }`) and default to 1.0, which leaves the friction to the surface's grip alone.

### Load and demand

On a road banked by θ into the turn:

```
a_normal = g·cos θ + v²·κ·sin θ + F_downforce / m      (load per unit mass)
a_lat    = | v²·κ·cos θ − g·sin θ |                     (lateral demand along the surface)
a_lat_max = μ_lat × a_normal
```

### Implementation

```
ellipse = √(1 − (a_lat / a_lat_max)²)        (0 at or past the cornering limit)
F_long_max = μ_long × m × a_normal × ellipse
```

**Physical interpretation:**
- On a straight (κ=0): Full traction available for acceleration/braking
- In a corner: Less traction available, must modulate throttle/brake
- At grip limit: No traction available for throttle/brake

Half the lateral grip in use leaves √¾ ≈ 87 % of the longitudinal grip, where a linear model would leave 50 %.

---

## Driver: Braking Envelope

### The quasi-steady-state lap

A lap at the limit is the lower of two speed curves along the track:

- **forward**: accelerating as hard as the engine and the tyres allow;
- **backward**: the fastest speed from which the car can still brake down to every corner's limit ahead.

The time integration is the forward curve, gear by gear. The backward curve (`BrakingEnvelope`) is computed once per track and car:

1. Nodes every `SimConfig::envelope_spacing` metres (0.5 m by default). Each node's cap is the corner speed of its segment, and of any segment that starts before the next node.
2. From the slowest cap of the lap, walk backwards once round the lap:

```
v_i = min(cap_i, √(v_{i+1}² + 2 · a_brake · h))
a_brake = (min(max_brake_force, F_long_max) + F_drag) / m
```

`a_brake` is taken at the lower of the two node speeds' estimates, so the envelope never asks for more than the car has.

### Following it

Each step the driver looks up the envelope where the car will be after the step. Full throttle if that stays under it; otherwise exactly the throttle or brake that lands the car on it:

```
F_needed = m · (v_allowed − v) / dt + F_drag
throttle = F_needed / F_engine         (F_needed ≥ 0)
brake    = −F_needed / max_brake_force (F_needed < 0)
```

Nothing clamps the speed. The car reaches each corner at its limit because it braked in time; a car forced in too fast would have no grip left to brake with and would carry its speed through.

---

## Laps and Timing

- **Standing lap** (default): from 0.1 m/s at the line.
- **Flying lap** (`SimConfig::flying_lap`): an untimed out lap from rest first; the clock starts as the car crosses the line at speed, as on a qualifying lap. On a closed circuit that is the speed of every lap after the first.
- Each telemetry frame carries the time of the state it holds. The lap time is interpolated within the step that crosses the line (and, on a flying lap, the step that crossed it at the start), so it hardly depends on the step size.
- Each step finds the car's segment once, by bisection over where the segments end (`SegmentIndex`), and hands it to the driver and the physics. A racing line cut into 2 m pieces has more than a thousand segments, and walking along them twice a step was most of a lap's cost.

---

## Numerical Integration

### Method: Semi-Implicit Euler

We use semi-implicit (symplectic) Euler integration:

```
a(t) = F_net(t) / m
v(t+dt) = v(t) + a(t) × dt
s(t+dt) = s(t) + v(t+dt) × dt   ← Uses NEW velocity
```

**Why semi-implicit Euler:**
- Better energy conservation than explicit Euler
- Computationally simple (single force evaluation per step)
- Deterministic and reproducible
- Stable for the stiffness levels in this simulation

### Timestep Selection

Default: `dt = 0.001s` (1 kHz)

Considerations:
- Smaller dt = more accurate but slower
- Larger dt = faster but may miss dynamics
- 1ms is a good balance for lap time simulation

---

## Engine RPM Calculation

RPM is calculated from wheel speed and gear ratios:

```
ω_wheel = v / r_wheel                    (rad/s)
ω_engine = ω_wheel × gear_ratio × final_drive
RPM = ω_engine × 60 / (2π)
```

---

## Units Summary

| Quantity | Symbol | Unit |
|----------|--------|------|
| Distance | s | meters (m) |
| Velocity | v | meters/second (m/s) |
| Acceleration | a | meters/second² (m/s²) |
| Force | F | Newtons (N) |
| Mass | m | kilograms (kg) |
| Torque | T | Newton-meters (Nm) |
| Time | t | seconds (s) |
| Curvature | κ | 1/meters (1/m) |
| Angle | θ | radians (rad) |

---

## References

1. Milliken, W. F., & Milliken, D. L. (1995). *Race Car Vehicle Dynamics*
2. Pacejka, H. B. (2012). *Tire and Vehicle Dynamics*
3. Siegler, B., & Crolla, D. (2002). "Lap Time Simulation for Racing Car Design"
