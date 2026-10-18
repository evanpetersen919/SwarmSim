include(GoogleTest)

file(GLOB_RECURSE SWARM_TEST_SOURCES CONFIGURE_DEPENDS
  unit/*.cpp property/*.cpp determinism/*.cpp integration/*.cpp)

add_executable(swarm_tests ${SWARM_TEST_SOURCES})
target_link_libraries(swarm_tests PRIVATE swarm::core swarm::format GTest::gtest_main)
target_include_directories(swarm_tests PRIVATE support)
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

# Architectural rules: run the invariants script on the real tree and on fixtures that must pass
# or fail. Needs bash, which Linux has and Windows has through Git for Windows.
find_program(SWARM_BASH bash HINTS "C:/Program Files/Git/bin" "C:/Program Files/Git/usr/bin"
             NO_DEFAULT_PATH)
if(NOT SWARM_BASH)
  find_program(SWARM_BASH bash)
endif()
if(SWARM_BASH)
  function(swarm_invariants_test name root expect_status expect_text)
    add_test(
      NAME ${name}
      COMMAND ${CMAKE_COMMAND}
        -DBASH=${SWARM_BASH}
        -DSCRIPT=${CMAKE_SOURCE_DIR}/scripts/check_core_invariants.sh
        -DROOT=${root}
        -DEXPECT_STATUS=${expect_status}
        "-DEXPECT_TEXT=${expect_text}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/integration/invariants_check.cmake)
  endfunction()

  set(FIXTURES ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/invariants)
  swarm_invariants_test(invariants_real_tree ${CMAKE_SOURCE_DIR} 0 "core invariants: ok")
  swarm_invariants_test(invariants_clean_estimator ${FIXTURES}/clean 0 "core invariants: ok")
  swarm_invariants_test(invariants_estimator_header_leak ${FIXTURES}/leak 1 "dynamics.hpp")
  swarm_invariants_test(invariants_estimator_source_leak ${FIXTURES}/leak_via_src 1 "scene.hpp")
else()
  message(STATUS "bash not found: invariants tests are skipped")
endif()
