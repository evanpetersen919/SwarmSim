# Warning and floating-point settings applied to every project target.
# Call swarm_configure_target(<target>) after add_library / add_executable.

function(swarm_configure_target target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /fp:precise /bigobj)
    if(SWARM_WERROR)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wconversion
      -ffp-contract=off)
    if(SWARM_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()
