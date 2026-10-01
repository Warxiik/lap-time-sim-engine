cmake_minimum_required(VERSION 3.14)
project(Lap_Time_Sim_Engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# When the engine is consumed via add_subdirectory() (e.g. as a git submodule),
# only the library is built by default; apps, tests and the benchmark are for
# standalone development.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(LTS_IS_TOP_LEVEL ON)
else()
    set(LTS_IS_TOP_LEVEL OFF)
endif()

option(LTS_BUILD_APPS "Build the interactive application" ${LTS_IS_TOP_LEVEL})
option(LTS_BUILD_TESTS "Build the unit tests" ${LTS_IS_TOP_LEVEL})
option(LTS_BUILD_BENCHMARK "Build the benchmark tool" ${LTS_IS_TOP_LEVEL})

include(FetchContent)

# JSON library for configuration files (reuse the parent project's if present)
if(NOT TARGET nlohmann_json::nlohmann_json)
    FetchContent_Declare(
            json
            URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    )
    FetchContent_MakeAvailable(json)
endif()

add_library(Lap_Time_Sim_Engine
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
        include/physics/tyre_model.hpp
        include/sim/simulator.hpp
        include/sim/step.hpp
        include/sim/driver_model.hpp
        include/sim/braking_envelope.hpp
        include/sim/segment_index.hpp
        include/sim/gear_map.hpp
        include/physics/tyre_condition.hpp
        include/models/fuel.hpp
        src/io/track_loader.cpp
        src/io/vehicle_loader.cpp
        src/io/telemetry_writer.cpp
        src/sim/simulator.cpp
        src/sim/step.cpp
        src/sim/driver_model.cpp
        src/sim/braking_envelope.cpp
        src/sim/segment_index.cpp
        src/sim/gear_map.cpp
        src/physics/tyre_condition.cpp
        src/physics/longitudinal.cpp
        src/physics/lateral.cpp
        src/physics/aero.cpp
        src/physics/tyre_model.cpp
        include/io/track_loader.hpp
        include/io/vehicle_loader.hpp
        include/sim/sim_config.hpp
        include/physics/vehicle_dynamics.hpp
        src/physics/vehicle_dynamics.cpp
)

target_include_directories(Lap_Time_Sim_Engine
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/io
)

target_link_libraries(Lap_Time_Sim_Engine
        PUBLIC nlohmann_json::nlohmann_json
)

if(WIN32)
    # M_PI is only defined by MSVC and MinGW headers with this macro.
    target_compile_definitions(Lap_Time_Sim_Engine PRIVATE _USE_MATH_DEFINES)
endif()

add_library(lts::engine ALIAS Lap_Time_Sim_Engine)

if(LTS_BUILD_APPS)
    add_executable(lap_time_sim_engine src/app/main.cpp)
    target_link_libraries(lap_time_sim_engine PRIVATE Lap_Time_Sim_Engine)
endif()

if(LTS_BUILD_TESTS)
    FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )

    # Prevents GoogleTest from overriding our compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(googletest)
    enable_testing()

    add_executable(sim_test
            tests/core/test_math.cpp
            tests/physics/test_lateral.cpp
            tests/physics/test_longitudinal.cpp
            tests/physics/test_tyre_model.cpp
            tests/sim/test_determinism.cpp
            tests/sim/test_braking_envelope.cpp
            tests/sim/test_lap.cpp
            tests/sim/test_driver_model.cpp
            tests/sim/test_segment_index.cpp
            tests/sim/test_gear_map.cpp
            tests/sim/test_stint.cpp
            tests/physics/test_tyre_condition.cpp
            tests/io/test_track_loader.cpp
            tests/io/test_vehicle_loader.cpp
    )

    target_link_libraries(sim_test
            PRIVATE
            gtest_main
            Lap_Time_Sim_Engine
    )

    add_test(NAME sim_test COMMAND sim_test)
endif()

if(LTS_BUILD_BENCHMARK)
    add_executable(benchmark tools/benchmark.cpp)
    target_link_libraries(benchmark PRIVATE Lap_Time_Sim_Engine)
endif()
