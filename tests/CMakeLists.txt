include(GoogleTest)

file(GLOB_RECURSE SWARM_TEST_SOURCES CONFIGURE_DEPENDS
  unit/*.cpp property/*.cpp determinism/*.cpp integration/*.cpp)

add_executable(swarm_tests ${SWARM_TEST_SOURCES})
target_link_libraries(swarm_tests PRIVATE swarm::core swarm::format GTest::gtest_main)
swarm_configure_target(swarm_tests)
gtest_discover_tests(swarm_tests)
