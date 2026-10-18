# Sanitizer selection through SWARM_SANITIZER (set by the CMake presets).

# Sanitizer presets build in Debug and run tens of times slower than release. SWARM_SANITIZED lets
# the heaviest tests use a smaller size there; release builds and the determinism job run them in
# full. See tests/support/test_scale.hpp.
if(NOT SWARM_SANITIZER STREQUAL "none")
  add_compile_definitions(SWARM_SANITIZED=1)
endif()

if(SWARM_SANITIZER STREQUAL "asan-ubsan")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
elseif(SWARM_SANITIZER STREQUAL "tsan")
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
elseif(NOT SWARM_SANITIZER STREQUAL "none")
  message(FATAL_ERROR "Unknown SWARM_SANITIZER '${SWARM_SANITIZER}'")
endif()
