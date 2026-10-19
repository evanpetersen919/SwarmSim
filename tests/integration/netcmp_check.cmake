# Runs the swarm_netcmp command line. Invoked by ctest with: -DNETCMP=<path to swarm_netcmp>

function(run_cmp expected_status expected_text)
  execute_process(
    COMMAND "${NETCMP}" ${ARGN}
    RESULT_VARIABLE status
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
  if(NOT status EQUAL expected_status)
    message(FATAL_ERROR
      "swarm_netcmp ${ARGN}\n  expected exit ${expected_status}, got ${status}\n  stdout: ${out}\n  stderr: ${err}")
  endif()
  if(NOT "${expected_text}" STREQUAL "")
    string(FIND "${out}${err}" "${expected_text}" at)
    if(at EQUAL -1)
      message(FATAL_ERROR "swarm_netcmp ${ARGN}\n  output lacks '${expected_text}'\n  stdout: ${out}\n  stderr: ${err}")
    endif()
  endif()
endfunction()

set(SMALL --endpoints 4 --ticks 60 --samples 50)

run_cmp(0 "identical deliveries: yes" ${SMALL})
run_cmp(0 "no commands" ${SMALL} --no-commands)
run_cmp(1 "identical deliveries: no" ${SMALL} --lose-every 7 --receive-timeout-ms 30)

run_cmp(2 "unknown option" --bogus 1)
run_cmp(2 "missing value" --ticks)
run_cmp(2 "invalid value" --ticks abc)
run_cmp(2 "endpoints must be" --endpoints 1)
run_cmp(2 "loss must be" ${SMALL} --loss 1.5)
run_cmp(0 "usage:" --help)
