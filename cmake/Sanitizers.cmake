# Sanitizer selection through SWARM_SANITIZER (set by the CMake presets).

if(SWARM_SANITIZER STREQUAL "asan-ubsan")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
elseif(SWARM_SANITIZER STREQUAL "tsan")
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
elseif(NOT SWARM_SANITIZER STREQUAL "none")
  message(FATAL_ERROR "Unknown SWARM_SANITIZER '${SWARM_SANITIZER}'")
endif()
