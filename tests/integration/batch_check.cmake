# Runs the swarm_batch command line. Invoked by ctest with:
#   -DBATCH=<path to swarm_batch> -DEXPERIMENTS=<experiments directory> -DWORKDIR=<scratch directory>

set(ONE "${WORKDIR}/batch_one.csv")
set(MANY "${WORKDIR}/batch_many.csv")
set(LOGS "${WORKDIR}/batch_logs")
file(REMOVE "${ONE}" "${MANY}" "${ONE}.meta" "${MANY}.meta")
file(REMOVE_RECURSE "${LOGS}")

function(run_batch expected_status expected_text)
  execute_process(
    COMMAND "${BATCH}" ${ARGN}
    RESULT_VARIABLE status
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
  if(NOT status EQUAL expected_status)
    message(FATAL_ERROR
      "swarm_batch ${ARGN}\n  expected exit ${expected_status}, got ${status}\n  stdout: ${out}\n  stderr: ${err}")
  endif()
  if(NOT "${expected_text}" STREQUAL "")
    string(FIND "${out}${err}" "${expected_text}" at)
    if(at EQUAL -1)
      message(FATAL_ERROR "swarm_batch ${ARGN}\n  output lacks '${expected_text}'\n  stdout: ${out}\n  stderr: ${err}")
    endif()
  endif()
endfunction()

run_batch(0 "wrote 8 rows" "${EXPERIMENTS}/smoke.yaml" --out "${ONE}" --jobs 1 --verify-every 1)
run_batch(0 "wrote 8 rows" "${EXPERIMENTS}/smoke.yaml" --out "${MANY}" --jobs 4 --verify-every 1
          --keep-logs "${LOGS}")

# The table is the same whatever the number of jobs.
file(READ "${ONE}" one)
file(READ "${MANY}" many)
if(NOT one STREQUAL many)
  message(FATAL_ERROR "the table differs between 1 and 4 jobs")
endif()
string(FIND "${one}" "experiment,cell,loss,drones,length,replicate,status,verified,scenario,seed," at)
if(NOT at EQUAL 0)
  message(FATAL_ERROR "unexpected header: ${one}")
endif()
string(REGEX MATCHALL "\n" newlines "${one}")
list(LENGTH newlines count)
if(NOT count EQUAL 9)
  message(FATAL_ERROR "expected a header and 8 rows, got ${count} lines")
endif()
string(FIND "${one}" "smoke,0,p00,n04,s04,0,ok,1,smoke.p00-n04-s04,1," at)
if(at EQUAL -1)
  message(FATAL_ERROR "the first row is not as expected: ${one}")
endif()

# The metadata file describes the run, and the logs were kept.
file(READ "${ONE}.meta" meta)
foreach(part "cells: 4" "seeds: 2 from 1" "runs: 8" "failed: 0" "experiment file:")
  string(FIND "${meta}" "${part}" at)
  if(at EQUAL -1)
    message(FATAL_ERROR "the metadata lacks '${part}': ${meta}")
  endif()
endforeach()
file(GLOB kept "${LOGS}/*.swlog")
list(LENGTH kept kept_count)
if(NOT kept_count EQUAL 8)
  message(FATAL_ERROR "expected 8 kept logs, got ${kept_count}")
endif()

# --seeds overrides the count.
run_batch(0 "wrote 4 rows" "${EXPERIMENTS}/smoke.yaml" --out "${MANY}" --seeds 1)

# Input errors are usage errors.
run_batch(2 "needs an experiment" --out "${ONE}")
run_batch(2 "needs an experiment" "${EXPERIMENTS}/smoke.yaml")
run_batch(2 "cannot read" "${EXPERIMENTS}/missing.yaml" --out "${ONE}")
run_batch(2 "unknown option" "${EXPERIMENTS}/smoke.yaml" --out "${ONE}" --bogus 1)
run_batch(2 "missing value" "${EXPERIMENTS}/smoke.yaml" --out)
run_batch(2 "invalid value" "${EXPERIMENTS}/smoke.yaml" --out "${ONE}" --jobs two)
run_batch(2 "--jobs must be" "${EXPERIMENTS}/smoke.yaml" --out "${ONE}" --jobs 0)
run_batch(2 "invalid value" "${EXPERIMENTS}/smoke.yaml" --out "${ONE}" --seeds 0)
run_batch(0 "usage:" --help)
