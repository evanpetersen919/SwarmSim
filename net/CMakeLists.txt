# swarm_net: the wire protocol and, in later commits, the socket transports and the socket channel.
# Real sockets and wall-clock time live here and in the apps, never in core/. core/ does not
# include anything from this directory.
add_library(swarm_net STATIC src/wire.cpp)
add_library(swarm::net ALIAS swarm_net)
target_include_directories(swarm_net PUBLIC include)
target_link_libraries(swarm_net PUBLIC swarm::core)
swarm_configure_target(swarm_net)
