# Runs scripts/check_core_invariants.sh on a directory and checks its exit status and output.
#   -DBASH=<bash> -DSCRIPT=<script> -DROOT=<directory> -DEXPECT_STATUS=<0|1> -DEXPECT_TEXT=<text>

execute_process(
  COMMAND "${BASH}" "${SCRIPT}" "${ROOT}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE out
  ERROR_VARIABLE err)

if(NOT status EQUAL EXPECT_STATUS)
  message(FATAL_ERROR
    "expected exit status ${EXPECT_STATUS}, got ${status}\nstdout: ${out}\nstderr: ${err}")
endif()
if(NOT EXPECT_TEXT STREQUAL "" AND NOT out MATCHES "${EXPECT_TEXT}")
  message(FATAL_ERROR "output does not mention '${EXPECT_TEXT}':\n${out}")
endif()
