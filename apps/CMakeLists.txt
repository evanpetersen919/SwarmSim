add_executable(swarm_sim swarm_sim.cpp)
target_link_libraries(swarm_sim PRIVATE swarm::core swarm::format)
target_compile_definitions(swarm_sim PRIVATE SWARM_VERSION="${PROJECT_VERSION}")
swarm_configure_target(swarm_sim)
