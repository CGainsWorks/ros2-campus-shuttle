# Plain CMake build of the controller core.
#
# This directory is intentionally free of ROS dependencies. It is built two ways:
#   1. add_subdirectory() from the ament package in ../CMakeLists.txt, so the
#      ROS 2 node and the Nav2 plugins link the same objects;
#   2. standalone in a bare toolchain image (docker/Dockerfile.bench), so the
#      comparison and the tests run without a ROS installation.
cmake_minimum_required(VERSION 3.16)
project(shuttle_control_core CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(shuttle_control_core
  src/path.cpp
  src/path_tracker.cpp
  src/pure_pursuit.cpp
  src/stanley.cpp
  src/diff_drive_model.cpp
  src/metrics.cpp
  src/scenarios.cpp
  src/bench_runner.cpp
)

target_include_directories(shuttle_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(shuttle_control_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(controller_bench bench/controller_bench.cpp)
target_link_libraries(controller_bench PRIVATE shuttle_control_core)

# Tests are only built standalone; the ament package registers its own.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR OR SHUTTLE_CORE_BUILD_TESTS)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_include_directories(test_core PRIVATE test)
  target_link_libraries(test_core PRIVATE shuttle_control_core)
  add_test(NAME core_tests COMMAND test_core)
endif()
