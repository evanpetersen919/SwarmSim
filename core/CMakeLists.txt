# swarm_format: log schema and hashing. No Eigen or yaml-cpp. These sources are also
# compiled directly into the Unreal plugin (see docs/decisions/0002-unreal-integration.md).
add_library(swarm_format STATIC format/src/hash.cpp format/src/log.cpp format/src/scene.cpp format/src/snapshot.cpp)
add_library(swarm::format ALIAS swarm_format)
target_include_directories(swarm_format PUBLIC format/include)
swarm_configure_target(swarm_format)

# swarm_core: simulation clock, RNG, and (later) drones, sensors, comms, estimation, control.
add_library(swarm_core STATIC
  sim/src/sim_clock.cpp
  sim/src/rng.cpp
  sim/src/delivery_queue.cpp
  sim/src/channel_model.cpp
  sim/src/deterministic_channel.cpp
  sim/src/lockstep_pool.cpp
  sim/src/simulation.cpp
  sim/src/demo_agent.cpp
  sim/src/run.cpp
  sim/src/dynamics.cpp
  sim/src/imu.cpp
  sim/src/terrain.cpp
  sim/src/obstacles.cpp
  sim/src/scene.cpp
  sim/src/geometry.cpp
  sim/src/lidar.cpp
  sim/src/weather.cpp
  sim/src/jammer.cpp
  sim/src/position_fix.cpp
  sim/src/estimation/pose_estimator.cpp
  sim/src/estimation/neighbor_message.cpp
  sim/src/estimation/neighbor_table.cpp
  sim/src/estimation/neighbor_predictor.cpp
  sim/src/control/formation.cpp
  sim/src/control/mission.cpp
  sim/src/control/formation_control.cpp
  sim/src/control/separation.cpp
  sim/src/control/fallback.cpp
  sim/src/control/command.cpp
  sim/src/control/drone_brain.cpp
  sim/src/drone_agent.cpp
  sim/src/ground_station.cpp)
add_library(swarm::core ALIAS swarm_core)
target_include_directories(swarm_core PUBLIC sim/include)
find_package(Threads REQUIRED)
target_link_libraries(swarm_core PUBLIC swarm_format swarm::eigen Threads::Threads)
swarm_configure_target(swarm_core)

add_subdirectory(scenario)
add_subdirectory(experiment)
