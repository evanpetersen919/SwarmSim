# swarm_scenario: reads scenario files (yaml-cpp) into a Scenario. Not part of core/format, so
# yaml-cpp stays out of the Unreal plugin.
add_library(swarm_scenario STATIC src/loader.cpp src/canonical.cpp src/world.cpp src/run.cpp src/metrics.cpp)
add_library(swarm::scenario ALIAS swarm_scenario)
target_include_directories(swarm_scenario PUBLIC include)
target_link_libraries(swarm_scenario PUBLIC swarm::core swarm::yamlcpp)
swarm_configure_target(swarm_scenario)
