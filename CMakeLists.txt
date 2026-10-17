cmake_minimum_required(VERSION 3.22)
project(SwarmSim VERSION 0.1.0 LANGUAGES CXX)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SWARM_BUILD_TESTS "Build the test suite" ON)
option(SWARM_BUILD_APPS "Build command-line programs" ON)
option(SWARM_WERROR "Treat warnings as errors in project targets" ON)
set(SWARM_SANITIZER "none" CACHE STRING "One of: none, asan-ubsan, tsan")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(Sanitizers)
include(Warnings)

add_subdirectory(core)
add_subdirectory(net)

if(SWARM_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(SWARM_BUILD_TESTS)
  include(Dependencies)
  enable_testing()
  add_subdirectory(tests)
endif()
