include(GoogleTest)

file(GLOB_RECURSE SWARM_TEST_SOURCES CONFIGURE_DEPENDS
  unit/*.cpp property/*.cpp determinism/*.cpp integration/*.cpp)

add_executable(swarm_tests ${SWARM_TEST_SOURCES})
target_link_libraries(swarm_tests PRIVATE swarm::core swarm::format GTest::gtest_main)
swarm_configure_target(swarm_tests)
gtest_discover_tests(swarm_tests)

if(TARGET swarm_sim)
  add_test(
    NAME cli_roundtrip
    COMMAND ${CMAKE_COMMAND}
      -DSIM=$<TARGET_FILE:swarm_sim>
      -DWORKDIR=${CMAKE_CURRENT_BINARY_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/integration/cli_roundtrip.cmake)
endif()
