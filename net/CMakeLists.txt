# swarm_net: the wire protocol, a portable socket wrapper, and in later commits the transports and
# the socket channel. Real sockets and wall-clock time live here and in the apps, never in core/.
# core/ does not include anything from this directory or any socket header; the invariants script
# checks both.
add_library(swarm_net STATIC src/wire.cpp src/socket.cpp src/udp_endpoints.cpp src/tcp_connections.cpp src/socket_channel.cpp)
add_library(swarm::net ALIAS swarm_net)
target_include_directories(swarm_net PUBLIC include)
target_link_libraries(swarm_net PUBLIC swarm::core)
if(WIN32)
  target_link_libraries(swarm_net PUBLIC ws2_32)
endif()
swarm_configure_target(swarm_net)
