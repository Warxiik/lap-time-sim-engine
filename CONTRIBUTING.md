# Contributing to Lap Time Simulation Engine

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## How to Contribute

### Reporting Bugs

- Open a [GitHub Issue](../../issues) with a clear description of the problem
- Include steps to reproduce the issue
- Specify your platform, compiler, and CMake version
- If possible, include the track/vehicle configuration that triggers the bug

### Suggesting Features

- Check the [Roadmap](README.md#roadmap) to see if it's already planned
- Open a GitHub Issue describing the feature and its use case

### Submitting Changes

1. Fork the repository
2. Create a feature branch from `main` (`git checkout -b feature/your-feature`)
3. Make your changes
4. Ensure all tests pass (`./sim_test` in the build directory)
5. Submit a pull request against `main`

## Development Setup

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

Run tests:

```bash
./sim_test
```

Run the benchmark to check for performance regressions:

```bash
./benchmark
```

## Code Style

- **Language**: C++17
- **Naming**: `snake_case` for functions and variables, `PascalCase` for types and structs
- **Headers**: Use `#pragma once` for include guards
- **Formatting**: Keep consistent with the existing codebase
- **Comments**: Explain *why*, not *what* -- especially for physics equations

## Guidelines

- Keep changes focused and minimal
- Maintain determinism guarantees -- avoid introducing RNG, adaptive timesteps, or heap allocations in the simulation loop
- Add tests for new physics models or data loaders
- Document any new physical assumptions in `docs/assumptions.md`
- Run the full test suite before submitting

## License

By contributing, you agree that your contributions will be licensed under the [MIT License](LICENSE).
