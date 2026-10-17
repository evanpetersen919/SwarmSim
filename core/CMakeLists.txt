# swarm_format: log schema and hashing. No Eigen or yaml-cpp. These sources are also
# compiled directly into the Unreal plugin (see docs/decisions/0002-unreal-integration.md).
add_library(swarm_format STATIC format/src/hash.cpp)
add_library(swarm::format ALIAS swarm_format)
target_include_directories(swarm_format PUBLIC format/include)
swarm_configure_target(swarm_format)

# swarm_core: simulation clock, RNG, and (later) drones, sensors, comms, estimation, control.
add_library(swarm_core STATIC
  sim/src/sim_clock.cpp
  sim/src/rng.cpp
  sim/src/delivery_queue.cpp
  sim/src/deterministic_channel.cpp)
add_library(swarm::core ALIAS swarm_core)
target_include_directories(swarm_core PUBLIC sim/include)
target_link_libraries(swarm_core PUBLIC swarm_format)
swarm_configure_target(swarm_core)
