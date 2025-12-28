cmake_minimum_required(VERSION 4.1)
project(Lap_Time_Sim_Engine)

set(CMAKE_CXX_STANDARD 14)

add_executable(Lap_Time_Sim_Engine
        src/app/main.cpp
        include/core/units.hpp
        include/core/math.hpp
        include/core/constants.hpp
        include/models/track.hpp
        include/models/vehicle.hpp
        include/models/tyre.hpp
        include/models/aero.hpp
        include/models/drivetrain.hpp
        include/models/car_state.hpp
        include/models/control.hpp
        include/models/telemetry.hpp
        include/physics/longitudinal.hpp
        include/physics/lateral.hpp
        include/physics/aero.hpp
        include/physics/tyres.hpp
        include/sim/simulator.hpp
        include/sim/step.hpp
        include/sim/driver_model.hpp
        src/io/track_loader.cpp
        src/io/telemetry_writer.cpp
        src/sim/simulator.cpp
        src/sim/step.cpp
        src/sim/driver_model.cpp
        src/physics/longitudinal.cpp
        src/physics/lateral.cpp
        src/physics/aero.cpp
        src/physics/tyres.cpp
        tests/physics/test_lateral.cpp
        tests/physics/test_longitudinal.cpp
        tests/sim/test_determinism.cpp
        tools/benchmark.cpp
)
