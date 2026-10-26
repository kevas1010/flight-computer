cmake_minimum_required(VERSION 3.20)
project(flight_computer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(FC_SANITIZE "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(FC_WERROR "Treat compiler warnings as errors" OFF)
option(FC_BUILD_TESTS "Build the GoogleTest suite" ON)

add_library(fc_warnings INTERFACE)
target_compile_options(fc_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
  $<$<BOOL:${FC_WERROR}>:-Werror>)

# No FMA contraction: log replay compares recomputed outputs against recorded ones,
# so floating-point results must not depend on optimisation level.
add_compile_options(-ffp-contract=off)

if(FC_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

# Deterministic flight software core: no sockets, no clocks, no threads.
add_library(fc_core
  src/common/crc32.cpp
  src/common/protocol.cpp
  src/common/stats.cpp
  src/fc/kalman.cpp
  src/fc/state_machine.cpp
  src/fc/guidance.cpp
  src/fc/faults.cpp
  src/fc/command_receiver.cpp
  src/fc/flight_computer.cpp
  src/fc/flight_log.cpp
  src/fc/replay.cpp)
target_include_directories(fc_core PUBLIC include)
target_link_libraries(fc_core PRIVATE fc_warnings)

# Vehicle physics, sensor models, fault injection and the in-process closed loop.
add_library(fc_sim_model
  src/sim/vehicle.cpp
  src/sim/closed_loop.cpp)
target_link_libraries(fc_sim_model PUBLIC fc_core PRIVATE fc_warnings)

# Linux-only runtime pieces: UDP sockets and the fixed-rate executive.
add_library(fc_runtime
  src/common/udp.cpp
  src/common/rt.cpp)
target_include_directories(fc_runtime PUBLIC include)
target_link_libraries(fc_runtime PRIVATE fc_warnings)

add_executable(fc_sim src/sim/main.cpp)
target_link_libraries(fc_sim PRIVATE fc_sim_model fc_runtime fc_warnings)

add_executable(flight_computer src/fc/main.cpp)
target_link_libraries(flight_computer PRIVATE fc_core fc_runtime fc_warnings)

add_executable(fc_replay src/tools/replay_main.cpp)
target_link_libraries(fc_replay PRIVATE fc_core fc_warnings)

add_executable(fc_closedloop src/tools/closedloop_main.cpp)
target_link_libraries(fc_closedloop PRIVATE fc_sim_model fc_warnings)

if(FC_BUILD_TESTS)
  enable_testing()
  find_package(GTest CONFIG QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(fc_tests
    tests/test_crc32.cpp
    tests/test_protocol.cpp
    tests/test_kalman.cpp
    tests/test_state_machine.cpp
    tests/test_guidance.cpp
    tests/test_faults.cpp
    tests/test_commands.cpp
    tests/test_closed_loop.cpp
    tests/test_replay.cpp
    tests/test_stats.cpp)
  target_link_libraries(fc_tests PRIVATE fc_core fc_sim_model GTest::gtest_main fc_warnings)
  target_compile_definitions(fc_tests PRIVATE
    FC_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

  include(GoogleTest)
  gtest_discover_tests(fc_tests DISCOVERY_TIMEOUT 60)
endif()
