include(FetchContent)

cmake_minimum_required(VERSION 3.14)
project(Lap_Time_Sim_Engine)

set(CMAKE_CXX_STANDARD 14)

add_library(Lap_Time_Sim_Engine
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
        include/io/track_loader.hpp
        include/io/vehicle_loader.hpp
        include/sim/sim_config.hpp
)

target_include_directories(Lap_Time_Sim_Engine
        PUBLIC ${CMAKE_SOURCE_DIR}/include
        PUBLIC ${CMAKE_SOURCE_DIR}/src
        PUBLIC ${CMAKE_SOURCE_DIR}/tests
        PUBLIC ${CMAKE_SOURCE_DIR}/tools
        PUBLIC ${CMAKE_SOURCE_DIR}/src/io
)

add_executable(lap_time_sim_engine src/app/main.cpp)
target_link_libraries(lap_time_sim_engine PRIVATE Lap_Time_Sim_Engine)

FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

# Prevents GoogleTest from overriding our compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)
enable_testing()

add_executable(sim_test
        tests/physics/test_lateral.cpp
        tests/physics/test_longitudinal.cpp
        tests/sim/test_determinism.cpp
)

target_link_libraries(sim_test
        PRIVATE
        gtest_main
        Lap_Time_Sim_Engine
)

add_test(NAME sim_tests COMMAND sim_tests)
