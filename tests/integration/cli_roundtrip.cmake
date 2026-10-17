# Runs the swarm_sim command line end to end. Invoked by ctest with:
#   -DSIM=<path to swarm_sim> -DWORKDIR=<scratch directory>

set(LOG "${WORKDIR}/cli_roundtrip.log")
file(REMOVE "${LOG}")

function(run_sim expected_status)
  execute_process(
    COMMAND "${SIM}" ${ARGN}
    RESULT_VARIABLE status
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
  if(NOT status EQUAL expected_status)
    message(FATAL_ERROR
      "swarm_sim ${ARGN}\n  expected exit ${expected_status}, got ${status}\n  stdout: ${out}\n  stderr: ${err}")
  endif()
  set(last_out "${out}" PARENT_SCOPE)
endfunction()

# Record on one thread, verify on eight.
run_sim(0 --record "${LOG}" --seed 7 --agents 12 --ticks 300 --threads 1
        --loss-model burst --loss 0.2 --mean-burst 4 --latency 2 --jitter 2 --rate-limit 3)
if(NOT EXISTS "${LOG}")
  message(FATAL_ERROR "no log was written")
endif()
run_sim(0 --replay "${LOG}" --verify --threads 8)

# A plain run prints the same line for any thread count.
run_sim(0 --seed 5 --agents 10 --ticks 200 --loss-model independent --loss 0.3 --threads 1)
set(one_thread "${last_out}")
run_sim(0 --seed 5 --agents 10 --ticks 200 --loss-model independent --loss 0.3 --threads 8)
if(NOT one_thread STREQUAL last_out)
  message(FATAL_ERROR "output differs between 1 and 8 threads:\n${one_thread}\n${last_out}")
endif()

# Failures: missing file, bad option, config flags with --replay, --replay without --verify.
run_sim(2 --replay "${WORKDIR}/does_not_exist.log" --verify)
run_sim(2 --bogus 1)
run_sim(2 --seed notanumber)
run_sim(2 --replay "${LOG}" --verify --seed 9)
run_sim(2 --replay "${LOG}")
run_sim(2 --threads 0)

# A log that is not a log fails verification with status 1.
file(WRITE "${WORKDIR}/not_a_log.bin" "this is not a swarm log")
run_sim(1 --replay "${WORKDIR}/not_a_log.bin" --verify)
