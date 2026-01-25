# Determinism in the Lap Time Simulator

This document explains what determinism means for simulation, why it matters, and how this simulator achieves it.

---

## What is Determinism?

A simulation is **deterministic** if:
> Running the same simulation with identical inputs always produces **bitwise identical** outputs.

This means:
- Same track + same car + same config = same lap time, every time
- Not just "approximately equal" but **exactly equal** to the last decimal place
- Results are reproducible across runs, machines, and time

---

## Why Determinism Matters

### 1. Valid Comparisons

When comparing two setups (e.g., high downforce vs low downforce), any difference in lap time must come from the setup change, not random variation.

```
Setup A: 1:23.456
Setup B: 1:23.789

Difference: 0.333s (100% attributable to setup change)
```

Without determinism, you cannot trust small differences.

### 2. Parameter Sweeps

Optimization requires sweeping parameters and comparing results:

```
for downforce in [2.5, 3.0, 3.5, 4.0]:
    lap_time = simulate(car.with_downforce(downforce))
    results.append(lap_time)
```

If simulations aren't deterministic, the results are meaningless noise.

### 3. Debugging and Validation

Deterministic simulations can be:
- Replayed exactly for debugging
- Validated against known results
- Compared across code versions

### 4. Caching and Memoization

If inputs determine outputs exactly, results can be cached:
```
cache_key = hash(track, vehicle, config)
if cache_key in cache:
    return cache[cache_key]
```

---

## Sources of Non-Determinism

Common causes of non-deterministic simulations:

### 1. Floating-Point Order of Operations

```cpp
// These may give different results:
double a = (x + y) + z;
double b = x + (y + z);
```

Floating-point addition is not associative due to rounding.

### 2. Uninitialized Variables

```cpp
double velocity;  // Contains garbage!
velocity += acceleration * dt;  // Non-deterministic
```

### 3. Random Number Generators

Any use of random numbers without fixed seeds introduces variation.

### 4. Timing-Dependent Code

```cpp
auto start = std::chrono::now();
// ... simulation ...
auto elapsed = std::chrono::now() - start;
```

Wall-clock time varies between runs.

### 5. Multi-Threading Race Conditions

Parallel execution with shared state can produce different results depending on thread scheduling.

### 6. Memory Allocation Timing

`std::vector` reallocation can cause timing variations that affect results if timing is measured.

---

## How This Simulator Achieves Determinism

### 1. Fixed Timestep

The simulation uses a constant timestep:

```cpp
SimConfig config;
config.dt = 0.001;  // Always 1ms, never varies
```

Variable timesteps (based on wall-clock time) would introduce non-determinism.

### 2. Deterministic Integration

Semi-implicit Euler integration is evaluated in a fixed order:

```cpp
// Always computed in this exact order:
double acceleration = net_force / mass;
state.v = state.v + acceleration * dt;
state.s = state.s + state.v * dt;
```

### 3. Initialized State

All state variables are explicitly initialized:

```cpp
CarState state{};
state.s = 0.0;
state.v = 0.1;  // Small initial velocity
state.a = 0.0;
state.gear = 1;
state.engine_rpm = 0.0;
```

### 4. No Random Numbers

The simulation contains no random number generation. All behavior is computed from deterministic physics equations.

### 5. Single-Threaded Execution

The simulation loop runs on a single thread:

```cpp
while (state.s < track.total_length) {
    // All computation is sequential
    control = driver.compute_control(state, vehicle, track);
    step::advance(state, vehicle, track, control, dt);
    telemetry.record(state, control, elapsed_time);
    elapsed_time += dt;
}
```

No race conditions or thread scheduling variations.

### 6. Pre-Allocated Memory

Telemetry uses `reserve()` to pre-allocate memory:

```cpp
telemetry.reserve(expected_frames);
```

This avoids reallocation during simulation, which could introduce timing variations.

### 7. Consistent Floating-Point Evaluation

All calculations use `double` precision and avoid:
- `long double` (platform-dependent precision)
- SIMD intrinsics (can vary by CPU)
- Fast-math compiler optimizations (can reorder operations)

---

## Testing Determinism

The test suite includes explicit determinism verification:

```cpp
TEST(SimulationDeterminism, IdenticalInputsProduceIdenticalOutputs) {
    // Run two identical simulations
    Simulator sim1(track, vehicle, config);
    Simulator sim2(track, vehicle, config);

    sim1.run();
    sim2.run();

    // Lap times must be EXACTLY equal (bitwise)
    ASSERT_DOUBLE_EQ(sim1.lap_time(), sim2.lap_time());

    // All telemetry must match exactly
    for (size_t i = 0; i < t1.size(); i += 50) {
        ASSERT_DOUBLE_EQ(t1.distance[i], t2.distance[i]);
        ASSERT_DOUBLE_EQ(t1.velocity[i], t2.velocity[i]);
    }
}
```

`ASSERT_DOUBLE_EQ` checks for bitwise equality, not approximate equality.

---

## Compiler Considerations

### Recommended Compiler Flags

```cmake
# Avoid fast-math optimizations that can break determinism
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-fast-math")
```

### Flags to Avoid

```
-ffast-math        # Allows reordering of operations
-funsafe-math-optimizations
-fassociative-math # Allows (a+b)+c != a+(b+c)
```

### Cross-Platform Determinism

For bitwise identical results across different machines:
- Use the same compiler version
- Use the same optimization level
- Avoid platform-specific intrinsics

Note: IEEE 754 compliance helps, but edge cases (denormals, NaN handling) can still vary.

---

## Limitations

### What IS Deterministic

- Same binary + same inputs = same outputs
- Repeated runs on the same machine
- Results can be exactly reproduced

### What MAY Vary

- Different compilers may produce different results
- Different optimization levels may affect results
- Different CPU architectures may have subtle differences
- Denormalized floating-point handling varies by platform

### Mitigation

For cross-platform reproducibility:
1. Compile with consistent settings
2. Flush denormals to zero: `_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON)`
3. Use fixed-point arithmetic for critical calculations (advanced)

---

## Best Practices for Maintaining Determinism

1. **Never use wall-clock time** in calculations
2. **Always initialize variables** explicitly
3. **Avoid dynamic memory allocation** in the hot loop
4. **Use fixed iteration order** for loops
5. **Test determinism explicitly** in the test suite
6. **Document any platform-specific behavior**

---

## Summary

| Requirement | Implementation |
|-------------|----------------|
| Fixed timestep | `config.dt = 0.001` (constant) |
| Initialized state | All fields explicitly set |
| No randomness | Deterministic physics only |
| Single-threaded | Sequential simulation loop |
| Pre-allocated memory | `telemetry.reserve()` |
| Explicit tests | `ASSERT_DOUBLE_EQ` for bitwise equality |

Determinism is not an accident — it requires deliberate design choices at every level of the simulation.
