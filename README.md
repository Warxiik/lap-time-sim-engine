# Lap Time Simulation Engine
This project implements a deterministic, fixed-step lap time simulation engine in modern C++.

The goal is not visual realism, but accurate and reproducible lap time estimation under explicit physical and numerical constraints. The simulation is distance-based (curvilinear coordinates), prioritising performance, determinism, and inspectability of all assumptions.

## Design Goals

- Deterministic results (bitwise identical across runs)
- Fixed-step integration
- No dynamic memory allocation in the hot loop
- Explicit physical and numerical assumptions
- Data-oriented design
- Performance measured and justified, not assumed

## Non-Goals

- Real-time graphics or visualisation
- Full vehicle dynamics (no suspension kinematics)
- Pacejka tyre modelling
- Driver-in-the-loop simulation
- Game-like behaviour or steering input

## Simulation Model

- Track represented as a sequence of segments defined by length and curvature
- Vehicle state evolves along distance, not Cartesian coordinates
- Cornering speed limited by lateral acceleration constraint
- Longitudinal acceleration limited by engine, braking, drag, and tyre grip
- Aerodynamic downforce modifies available tyre grip
- Tyres modelled with simplified grip and degradation parameters

## Project Structure

include/    - Public headers and data definitions  
src/        - Implementation  
physics/    - Stateless force and constraint calculations  
sim/        - Simulation orchestration and integration loop  
models/     - Pure data structures  
tests/      - Determinism and physics unit tests  
data/       - Track and vehicle input files  
tools/      - Benchmarking and profiling utilities

## Determinism & Correctness

- Fixed integration timestep
- No random numbers in the simulation loop
- No heap allocation during stepping
- Identical inputs produce identical outputs across runs
- Step-size sensitivity tested

## Performance

The simulator is designed to run thousands of laps per second on a single core, enabling large parameter sweeps and strategy evaluation.

Performance is validated using:
- Compiler optimisation flags (-O3, LTO)
- Profiling tools (perf, Tracy)
- Benchmarks in tools/

## Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Run

```bash
./lap_sim data/tracks/monza.csv data/vehicles/f1_like.json
```

## Testing

Unit tests validate:
- Lateral and longitudinal force limits
- Numerical stability
- Deterministic lap time output

## Roadmap

- Tyre temperature dynamics
- Elevation and gradient effects
- Simple energy recovery system
- Parallel strategy simulation

## Notes

This project is intended as an engineering exercise focused on correctness and performance, not a complete vehicle dynamics model.
