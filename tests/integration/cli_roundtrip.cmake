# Runs the swarm_sim command line end to end. Invoked by ctest with:
#   -DSIM=<path to swarm_sim> -DWORKDIR=<scratch directory> -DSCENARIOS=<scenarios directory>

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

# Jammers: pulsed local zone and a global one that affects senders too.
run_sim(0 --record "${LOG}" --seed 11 --agents 10 --ticks 200 --threads 2
        --jam 40,0,0,25,pulsed,50,150,20,0.5,0,1,receiver
        --jam 0,0,0,inf,constant,100,120,0,0.5,0,0.5,either)
run_sim(0 --replay "${LOG}" --verify --threads 6)
run_sim(2 --jam 1,2,3 --ticks 10)
run_sim(2 --jam 0,0,0,10,loud,0,100,0,0.5,0,1,receiver --ticks 10)

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

# Scenario runs: record on one thread, verify on eight; the printed hash is thread-invariant.
set(SLOG "${WORKDIR}/cli_scenario.log")
file(REMOVE "${SLOG}")
run_sim(0 run "${SCENARIOS}/formation_jam.yaml" --out "${SLOG}" --threads 1)
if(NOT EXISTS "${SLOG}")
  message(FATAL_ERROR "no scenario log was written")
endif()
set(scenario_one "${last_out}")
run_sim(0 --replay "${SLOG}" --verify --threads 8)
run_sim(0 run "${SCENARIOS}/formation_jam.yaml" --out "${WORKDIR}/cli_scenario8.log" --threads 8)
string(REPLACE "cli_scenario8.log" "cli_scenario.log" scenario_eight "${last_out}")
if(NOT scenario_one STREQUAL scenario_eight)
  message(FATAL_ERROR "scenario output differs between 1 and 8 threads:
${scenario_one}
${scenario_eight}")
endif()

# Scenario failures: missing file, an invalid scenario, no --out, an unknown option.
run_sim(2 run "${WORKDIR}/no_such_scenario.yaml" --out "${SLOG}")
file(WRITE "${WORKDIR}/bad_scenario.yaml" "version: 1
seed: 1
duration_s: 5
colour: red
")
run_sim(2 run "${WORKDIR}/bad_scenario.yaml" --out "${SLOG}")
run_sim(2 run "${SCENARIOS}/minimal.yaml")
run_sim(2 run "${SCENARIOS}/minimal.yaml" --out "${SLOG}" --bogus)

# Metrics: a CSV of one header row and one data row for a recorded scenario log.
set(EXPECTED_HEADER "scenario,seed,drones,dt_s,ticks,duration_s,formation_rmse_m,formation_rmse_settled_m,alive_at_end,min_separation_m,collision_episodes,recovery_count,recovery_longest_s,recovery_censored,messages_sent,messages_delivered,messages_in_flight,dropped_loss,dropped_rate,dropped_jam,delivery_ratio,latency_p50_s,latency_p95_s")
set(CSV "${WORKDIR}/cli_metrics.csv")
file(REMOVE "${CSV}")
run_sim(0 metrics "${SLOG}" --csv "${CSV}")
if(NOT EXISTS "${CSV}")
  message(FATAL_ERROR "no metrics CSV was written")
endif()
file(STRINGS "${CSV}" csv_lines)
list(LENGTH csv_lines csv_line_count)
if(NOT csv_line_count EQUAL 2)
  message(FATAL_ERROR "expected a header and one row, got ${csv_line_count} lines")
endif()
list(GET csv_lines 0 csv_header)
list(GET csv_lines 1 csv_row)
if(NOT csv_header STREQUAL EXPECTED_HEADER)
  message(FATAL_ERROR "unexpected CSV header:
${csv_header}")
endif()
if(NOT csv_row MATCHES "^formation_jam,7,8,0\.02,3000,60,")
  message(FATAL_ERROR "unexpected CSV row:
${csv_row}")
endif()

# Without --csv the same two lines go to standard output.
run_sim(0 metrics "${SLOG}")
if(NOT last_out MATCHES "^scenario,seed,drones,")
  message(FATAL_ERROR "metrics did not print the CSV header:
${last_out}")
endif()

# Metrics failures: a missing file, a file that is not a log, a demo log, and bad arguments.
run_sim(2 metrics "${WORKDIR}/does_not_exist.log")
run_sim(2 metrics "${WORKDIR}/not_a_log.bin")
run_sim(2 metrics "${LOG}")
run_sim(2 metrics)
run_sim(2 metrics "${SLOG}" --csv)
run_sim(2 metrics "${SLOG}" --bogus)

# A scenario with commands: the station is not a drone, and the log verifies on another thread count.
set(CLOG "${WORKDIR}/cli_commands.log")
file(REMOVE "${CLOG}")
run_sim(0 run "${SCENARIOS}/hold_resume.yaml" --out "${CLOG}" --threads 1)
run_sim(0 --replay "${CLOG}" --verify --threads 4)
run_sim(0 metrics "${CLOG}")
if(NOT last_out MATCHES "\nhold_resume,11,6,")
  message(FATAL_ERROR "unexpected metrics row for the commands scenario:\n${last_out}")
endif()

# Running over real sockets: the log is byte-identical to the simulated run's, it verifies on the
# simulated channel, and bad transport options are usage errors.
set(SOCK_LOG "${WORKDIR}/cli_sockets.log")
file(REMOVE "${SOCK_LOG}")
run_sim(0 run "${SCENARIOS}/formation_jam.yaml" --out "${SOCK_LOG}" --threads 2 --transport sockets)
if(NOT last_out MATCHES "network drops: 0")
  message(FATAL_ERROR "the sockets run did not report zero network drops:\n${last_out}")
endif()
run_sim(0 --replay "${SOCK_LOG}" --verify --threads 3)
execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${SLOG}" "${SOCK_LOG}"
                RESULT_VARIABLE log_differs)
if(NOT log_differs EQUAL 0)
  message(FATAL_ERROR "the log recorded over sockets differs from the simulated log")
endif()

set(SOCK_CLOG "${WORKDIR}/cli_sockets_commands.log")
file(REMOVE "${SOCK_CLOG}")
run_sim(0 run "${SCENARIOS}/hold_resume.yaml" --out "${SOCK_CLOG}" --transport sockets)
execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${CLOG}" "${SOCK_CLOG}"
                RESULT_VARIABLE commands_differ)
if(NOT commands_differ EQUAL 0)
  message(FATAL_ERROR "the commands log recorded over sockets differs from the simulated log")
endif()

run_sim(0 run "${SCENARIOS}/minimal.yaml" --out "${WORKDIR}/cli_simulated_flag.log" --transport simulated)
run_sim(2 run "${SCENARIOS}/minimal.yaml" --out "${WORKDIR}/x.log" --transport carrier-pigeon)
run_sim(2 run "${SCENARIOS}/minimal.yaml" --out "${WORKDIR}/x.log" --transport)
run_sim(2 run "${SCENARIOS}/minimal.yaml" --out "${WORKDIR}/x.log" --transport sockets
        --receive-timeout-ms abc)
run_sim(2 run "${SCENARIOS}/minimal.yaml" --out "${WORKDIR}/x.log" --transport sockets
        --receive-timeout-ms -5)

# export-scene: the terrain and obstacles of a scenario log as a scene file for the Unreal viewer.
set(SCENE "${WORKDIR}/cli_scene.swscene")
file(REMOVE "${SCENE}")
run_sim(0 export-scene "${SLOG}" --out "${SCENE}")
if(NOT EXISTS "${SCENE}")
  message(FATAL_ERROR "no scene file was written")
endif()
if(NOT last_out MATCHES "wrote scene for formation_jam: 125 x 125 cells, 0 obstacles, hash=[0-9a-f]+")
  message(FATAL_ERROR "unexpected export-scene output:
${last_out}")
endif()
file(READ "${SCENE}" scene_head LIMIT 4)
if(NOT scene_head STREQUAL "SWSC")
  message(FATAL_ERROR "the scene file does not start with the scene magic")
endif()
# The same log gives the same scene file.
set(SCENE2 "${WORKDIR}/cli_scene2.swscene")
run_sim(0 export-scene "${SLOG}" --out "${SCENE2}")
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files "${SCENE}" "${SCENE2}" RESULT_VARIABLE same)
if(NOT same EQUAL 0)
  message(FATAL_ERROR "two exports of the same log differ")
endif()
run_sim(2 export-scene "${WORKDIR}/does_not_exist.log" --out "${SCENE}")
run_sim(2 export-scene "${WORKDIR}/not_a_log.bin" --out "${SCENE}")
run_sim(2 export-scene "${LOG}" --out "${SCENE}")
run_sim(2 export-scene "${SLOG}")
run_sim(2 export-scene --out "${SCENE}")
run_sim(2 export-scene "${SLOG}" --out)
run_sim(2 export-scene "${SLOG}" --out "${SCENE}" --bogus)

# A scenario whose scene comes from that file: it runs, its log holds the scene's hash, the log
# verifies, and exporting the scene from that log gives back the same file.
set(FILE_SCENARIO "${WORKDIR}/cli_scene_scenario.yaml")
file(WRITE "${FILE_SCENARIO}"
  "version: 1
seed: 3
duration_s: 10
scene:
  source: file
  path: \"${SCENE}\"
"
  "swarm:
  drones: 4
mission:
  waypoints:
    - [0, 0, 20]
")
set(FLOG "${WORKDIR}/cli_scene_file.log")
file(REMOVE "${FLOG}")
run_sim(0 run "${FILE_SCENARIO}" --out "${FLOG}" --threads 1)
run_sim(0 --replay "${FLOG}" --verify --threads 4)
set(SCENE3 "${WORKDIR}/cli_scene3.swscene")
run_sim(0 export-scene "${FLOG}" --out "${SCENE3}")
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files "${SCENE}" "${SCENE3}" RESULT_VARIABLE same)
if(NOT same EQUAL 0)
  message(FATAL_ERROR "the scene exported from a run on a scene file differs from that file")
endif()
# The file is the scene: a missing file is a usage error.
file(WRITE "${WORKDIR}/cli_scene_missing.yaml"
  "version: 1
seed: 3
duration_s: 10
scene:
  source: file
  path: \"${WORKDIR}/no_such.swscene\"
"
  "swarm:
  drones: 4
mission:
  waypoints:
    - [0, 0, 20]
")
run_sim(2 run "${WORKDIR}/cli_scene_missing.yaml" --out "${FLOG}")
