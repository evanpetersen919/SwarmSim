# Wire protocol and UDP/TCP channel. Header-only placeholder until the first source file exists.
add_library(swarm_net INTERFACE)
add_library(swarm::net ALIAS swarm_net)
target_include_directories(swarm_net INTERFACE include)
target_link_libraries(swarm_net INTERFACE swarm::core)
