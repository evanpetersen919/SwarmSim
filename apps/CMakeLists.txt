add_executable(swarm_sim swarm_sim.cpp)
target_link_libraries(swarm_sim PRIVATE swarm::scenario swarm::net swarm::core swarm::format)
target_compile_definitions(swarm_sim PRIVATE SWARM_VERSION="${PROJECT_VERSION}")
swarm_configure_target(swarm_sim)

add_executable(swarm_bench swarm_bench.cpp)
target_link_libraries(swarm_bench PRIVATE swarm::core swarm::format)
swarm_configure_target(swarm_bench)

add_executable(swarm_netcmp swarm_netcmp.cpp)
target_link_libraries(swarm_netcmp PRIVATE swarm::net swarm::core swarm::format)
swarm_configure_target(swarm_netcmp)
