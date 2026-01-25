# Assumptions and Simplifications

This document lists the assumptions and simplifications made in the Lap Time Simulation Engine, along with their justifications and potential impacts.

---

## Vehicle Model Assumptions

### 1. Point Mass Model

**Assumption:** The vehicle is treated as a point mass with no physical dimensions.

**Justification:**
- Dramatically simplifies dynamics calculations
- Sufficient for lap time estimation (not handling simulation)
- Standard approach in production lap time simulators

**Impact:**
- No yaw dynamics (rotation about vertical axis)
- No roll or pitch motion
- Cannot simulate oversteer/understeer characteristics

**Future enhancement:** Add yaw inertia and slip angle model.

---

### 2. No Weight Transfer

**Assumption:** Weight distribution remains constant regardless of acceleration.

**Justification:**
- Simplifies force calculations
- Weight transfer effects are secondary for lap time estimation
- Would require suspension model for accuracy

**Impact:**
- Front/rear grip split not modeled
- Braking and acceleration grip limits are symmetric
- Cannot optimize brake bias

**Real behavior:**
- Under braking: weight shifts forward, more front grip
- Under acceleration: weight shifts rearward, more rear grip
- In corners: weight shifts to outside wheels

**Future enhancement:** Add longitudinal weight transfer model:
```
delta_Fz = m * a * h_cg / wheelbase
```

---

### 3. No Suspension Dynamics

**Assumption:** Perfect tire-road contact at all times.

**Justification:**
- Suspension effects are secondary for smooth tracks
- Would require complex spring/damper model
- Most lap time simulators omit suspension

**Impact:**
- No bump/kerb response
- No ride height changes with speed
- No suspension-induced grip variations

---

## Tire Model Assumptions

### 4. Linear Friction Coefficient

**Assumption:** Tire grip is constant regardless of slip angle or load.

**Justification:**
- Simplifies grip calculations significantly
- Reasonable for estimating peak grip
- Avoids need for Pacejka tire model parameters

**Impact:**
- No load sensitivity (grip doesn't decrease with higher loads)
- No slip angle optimization
- Cannot model tire compound differences accurately

**Real behavior:**
- Grip decreases at very high loads (load sensitivity)
- Optimal slip angle exists (typically 6-12 degrees)
- Different compounds have different grip characteristics

**Future enhancement:** Implement simplified load sensitivity:
```
mu_effective = mu_base * (F_z_ref / F_z)^n
```

---

### 5. Linear Friction Circle

**Assumption:** Combined grip follows a linear relationship rather than circular.

**Formula used:**
```
traction_available = 1 - (a_lat / a_lat_max)
```

**True circular model:**
```
a_long^2 + a_lat^2 <= a_max^2
```

**Impact:**
- Slightly pessimistic combined grip prediction
- Approximately 10-15% error at high combined loads
- Simpler to compute

---

### 6. No Tire Thermal Model

**Assumption:** Tires operate at optimal temperature always.

**Justification:**
- Thermal modeling requires detailed tire data
- Adds significant complexity
- Out-lap/in-lap effects are secondary

**Impact:**
- No tire warm-up phase
- No degradation over stint
- Cannot optimize tire strategy

---

## Aerodynamic Assumptions

### 7. Constant Aerodynamic Coefficients

**Assumption:** Cd and Cl remain constant regardless of ride height, yaw, or configuration.

**Justification:**
- Aerodynamic maps require CFD/wind tunnel data
- Constant coefficients are sufficient for basic simulation
- DRS/configuration changes can be modeled with different vehicle files

**Impact:**
- No DRS modeling within simulation
- No sensitivity to ride height
- No yaw-induced drag changes

---

### 8. Sea Level Air Density

**Assumption:** Air density = 1.225 kg/m^3 (sea level, 15C)

**Impact:**
- Slightly optimistic for high-altitude circuits (Mexico City: ~22% less dense)
- Could add 1-2 seconds at altitude

**Future enhancement:** Accept air density as track parameter.

---

## Powertrain Assumptions

### 9. Instant Gear Changes

**Assumption:** Gear changes are instantaneous with no power interruption.

**Justification:**
- Modern F1/racing cars have seamless shift gearboxes
- Shift times are typically 10-50ms
- Minor impact on lap time

**Impact:**
- Slightly optimistic acceleration
- No clutch slip modeling

---

### 10. No Rev Limiter

**Assumption:** Engine doesn't hit rev limiter (driver always shifts before).

**Impact:**
- Could allow unrealistic RPM in edge cases
- Gear selection algorithm handles this in practice

---

### 11. Constant Drivetrain Efficiency

**Assumption:** Efficiency is constant across all conditions.

**Real behavior:**
- Efficiency varies with load and speed
- Typically 85-95% for racing transmissions

**Impact:** Minor, as efficiency variation is typically less than 5%.

---

## Driver Model Assumptions

### 12. Perfect Knowledge

**Assumption:** Driver knows exact grip limits and track layout.

**Justification:**
- Simulating "the car" not "the driver"
- Represents theoretical optimal performance
- Human factors are out of scope

**Impact:**
- Results represent car potential, not realistic lap times
- No driver errors or inconsistency

---

### 13. Bang-Bang Control

**Assumption:** Driver uses full throttle or full brake, no partial inputs.

**Justification:**
- Simplifies control logic
- Near-optimal for many situations
- Easy to implement and debug

**Impact:**
- No trail braking (partial brake while turning)
- No throttle modulation for traction control
- Typically 1-3% slower than optimal

**Future enhancement:** Implement quasi-static optimal control.

---

### 14. Centerline Racing

**Assumption:** Vehicle follows track centerline exactly.

**Real behavior:**
- Optimal racing line uses full track width
- Can significantly shorten corner distance
- Affects corner entry/exit speeds

**Impact:**
- Lap times are pessimistic (longer path)
- Approximately 2-5% slower than optimal line

---

## Track Model Assumptions

### 15. Discrete Constant-Curvature Segments

**Assumption:** Track is composed of segments with constant curvature.

**Justification:**
- Simplifies track representation
- Standard approach in lap simulation
- Can approximate any track with enough segments

**Impact:**
- Curvature transitions are discontinuous
- May miss optimal braking points slightly
- Adequate resolution with 50+ segments

---

### 16. Flat Track (No Elevation)

**Assumption:** Track is flat with no elevation changes.

**Justification:**
- Elevation requires 3D track model
- Adds complexity to force calculations
- Many circuits are relatively flat

**Impact:**
- Cannot model Spa's Eau Rouge correctly
- Gravity assist on downhills not captured
- Could be 1-3 seconds off on hilly circuits

**Future enhancement:** Add elevation profile to track model.

---

### 17. Constant Grip Surface

**Assumption:** Track surface grip is constant (or constant per segment).

**Real behavior:**
- Grip varies with rubber buildup
- Temperature affects grip
- Rain creates variable conditions

**Impact:**
- Cannot model track evolution
- No wet weather simulation

---

## Summary Table

| Assumption | Impact on Lap Time | Difficulty to Fix |
|------------|-------------------|-------------------|
| Point mass | Low | High |
| No weight transfer | Low-Medium | Medium |
| Linear friction | Low | Low |
| No thermal model | Medium (for stints) | High |
| Constant aero | Low | Medium |
| Bang-bang control | Low-Medium | Medium |
| Centerline racing | Medium | High |
| No elevation | Low-High (track dependent) | Medium |

---

## When Are These Assumptions Valid?

The model is most accurate when:
- Comparing relative performance (car A vs car B)
- Smooth, flat circuits with clear corners
- Single-lap performance (no tire degradation)
- High-grip conditions

The model is less accurate for:
- Absolute lap time prediction
- Circuits with significant elevation (Spa, COTA)
- Long stints with tire wear
- Changeable weather conditions
