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

### Brake Force

In the simplified model, maximum brake force is a constant parameter representing the combined limit of:
- Brake disc/caliper capacity
- Brake-by-wire system limits

```
F_brake = brake_pedal × max_brake_force
```

### Net Longitudinal Force

```
F_net = F_drive - F_brake - F_drag
```

The acceleration is then:
```
a = F_net / m
```

---

## Lateral Dynamics

### Maximum Cornering Speed

The maximum speed through a corner is limited by available lateral grip:

```
v_max = √(a_lat_max / κ)
```

Where:
- `a_lat_max` = Maximum lateral acceleration (m/s²)
- `κ` = Track curvature (1/radius, in 1/m)

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

## Tire Model (Friction Circle)

### Concept

Tires have a limited total grip budget shared between:
- Lateral force (cornering)
- Longitudinal force (acceleration/braking)

This is visualized as a "friction circle" where the radius represents maximum grip.

### Implementation

We use a **linear friction circle** model:

```
traction_available = 1 - (a_lat_current / a_lat_max)
```

Where:
- `a_lat_current = v² × κ` (required centripetal acceleration)
- `a_lat_max` = Maximum available lateral acceleration

This scales the available longitudinal force:
```
F_tire_limited = F_tire × traction_available
```

**Physical interpretation:**
- On a straight (κ=0): Full traction available for acceleration/braking
- In a corner: Less traction available, must modulate throttle/brake
- At grip limit: No traction available for throttle/brake

### Limitations

The linear model is slightly pessimistic compared to a true circular model:
```
# True circular model:
F_long_available = √(F_max² - F_lat²)

# Our linear model:
F_long_available = F_max × (1 - F_lat/F_max)
```

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
