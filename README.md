# Lap Time Simulation Engine

A deterministic, fixed-timestep lap time simulation engine written in modern C++17.

This project implements a point-mass vehicle dynamics model for accurate and reproducible lap time estimation. The simulation uses curvilinear coordinates (distance-based), prioritizing performance, determinism, and full transparency of all physical assumptions.

## Features

- **Deterministic simulation** - Bitwise identical results across runs
- **Point-mass vehicle model** - Simplified but physically grounded
- **Friction ellipse tyre model** - Lateral and longitudinal grip from the tyres and the surface, shared in an ellipse; the engine and the brakes are limited by it
- **Aerodynamic forces** - Drag and downforce affecting grip and top speed; corner speeds are aero-consistent (downforce at the corner's own speed), with banking
- **Quasi-steady-state driver** - Follows a backward braking envelope with graded throttle and brake; nothing clamps the speed
- **Standing and flying laps** - From rest, or timed from the line after an out lap
- **Real-world data** - Includes F1 tracks and vehicle configurations
- **Interactive CLI** - Select tracks and vehicles at runtime
- **Performance benchmarking** - Measure throughput and verify determinism

## Quick Start

### Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Run Simulation

```bash
./lap_time_sim_engine
```

The program will scan the `data/` directory and present an interactive menu to select a track and vehicle.

### Run Tests

```bash
./sim_test
```

### Run Benchmark

```bash
./benchmark [iterations]
```

Or use the helper script:

```powershell
# Windows (PowerShell)
.\tools\perf_runner.ps1 -Release -Iterations 100
```

## Project Structure

```
├── include/
│   ├── core/           # Units, math utilities, physical constants
│   ├── models/         # Data structures (track, vehicle, telemetry)
│   ├── physics/        # Force and constraint calculations
│   ├── sim/            # Simulator, driver model, integration
│   └── io/             # File loaders
├── src/
│   ├── app/            # Main application entry point
│   ├── physics/        # Physics implementations
│   ├── sim/            # Simulation loop and driver logic
│   └── io/             # Track/vehicle loaders, telemetry writer
├── tests/              # GoogleTest unit tests
├── tools/              # Benchmark and profiling utilities
├── data/
│   ├── tracks/         # Track definitions (CSV)
│   └── vehicles/       # Vehicle configurations (JSON)
└── docs/               # Technical documentation
```

## Included Data

### Tracks
- Monaco (3.337 km) - Tight street circuit
- Monza (5.793 km) - High-speed temple
- Silverstone (5.891 km) - Fast and flowing
- Spa-Francorchamps (7.004 km) - Legendary circuit
- Suzuka (5.807 km) - Figure-eight layout

### Vehicles
- 2024 F1 Car - Full downforce configuration
- Formula 2 Car - Spec series vehicle
- Porsche 911 GT3 R - GT racing spec
- Porsche 911 Turbo S - High-performance road car

## Physics Model

The simulation implements a point-mass model with:

- **Aerodynamics**: Drag force `F = 0.5 × ρ × Cd × A × v²` and downforce
- **Lateral dynamics**: Steady corner speed with downforce and banking, `v² = g(μ cos θ + sin θ) / (κ(cos θ − μ sin θ) − μ ρ|Cl|A/2m)`
- **Longitudinal dynamics**: Engine torque curves, gear ratios, braking, all within the tyres' grip
- **Tyre model**: Friction ellipse, with the tyres' lateral and longitudinal friction (`tyre` in the vehicle JSON) times the surface's grip
- **Driver**: A backward braking envelope computed once per lap; the time integration is the forward pass
- **Stints** (optional): laps one after another, burning fuel (brake-specific consumption) and heating, wearing and inflating each axle's tyres, which lose grip off their temperature, pressure and tread
- **Integration**: Semi-implicit Euler with fixed 1ms timestep

See [docs/physics_model.md](docs/physics_model.md) for complete equations.

## Determinism

The simulator guarantees identical outputs for identical inputs:

- Fixed integration timestep (no adaptive stepping)
- No random number generation
- No floating-point non-determinism (consistent operation order)
- No heap allocation in the simulation loop

See [docs/determinism.md](docs/determinism.md) for implementation details.

## Performance

Measured with MinGW-w64 GCC 14.2, Release, 1 ms step, on one core:

| Lap | Time per lap |
|--------|-------|
| `benchmark`: a 14-segment, 5 km track, 83 s lap | about 24 ms (about 3,500 times real time) |
| a 2.3 km racing line in 1,142 segments of 2 m, a 60–68 s lap | about 13–15 ms |

The time grows with the steps a lap takes, not with the segments: each step finds its segment by bisection. `benchmark [iterations]` measures it and checks determinism.

This enables large-scale parameter sweeps and strategy optimization.

## Documentation

- [Physics Model](docs/physics_model.md) - All equations and formulas
- [Assumptions](docs/assumptions.md) - Documented simplifications
- [Determinism](docs/determinism.md) - How determinism is achieved

## Dependencies

- C++17 compiler (GCC, Clang, MSVC)
- CMake 3.14+
- [nlohmann/json](https://github.com/nlohmann/json) (fetched automatically)
- [GoogleTest](https://github.com/google/googletest) (fetched automatically)

## Roadmap

- [ ] Tyre temperature and degradation dynamics
- [ ] Elevation and track gradient effects
- [ ] Energy recovery system (ERS) modeling
- [ ] Fuel load and consumption
- [ ] Multi-lap race simulation
- [ ] Parallel parameter sweeps

## License

This project is an engineering exercise for educational purposes.
