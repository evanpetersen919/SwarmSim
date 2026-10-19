# swarm_experiment: experiment files, their expansion into cells, and the batch driver. Uses
# yaml-cpp through swarm_scenario.
add_library(swarm_experiment STATIC src/experiment.cpp)
add_library(swarm::experiment ALIAS swarm_experiment)
target_include_directories(swarm_experiment PUBLIC include)
target_link_libraries(swarm_experiment PUBLIC swarm::scenario)
swarm_configure_target(swarm_experiment)
