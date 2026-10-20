# Phase 7: Batch runs and analysis

Status: complete. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md), [MODELS.md](../MODELS.md), [phase 5](phase-05-scenarios-and-metrics.md).

## Goal

One command expands an experiment description into its cells, runs every cell for every seed, and writes a results table. A second command turns that table into summary statistics with confidence intervals and figures. The same description and the same build give the same table, whatever the machine's core count or the order cells finish in. A small set of validation checks shows the fog model against a published curve.

## Non-goals

A cluster scheduler, resumable multi-day runs, Parquet, a database, an interactive dashboard, hypothesis testing across the whole matrix, and a claim that the results describe real drones. Tick time against drone count is a benchmark, not a log metric, and stays in `swarm_bench` (Phase 9 records it).

## What exists and what is missing

Single scenarios run, record a log, verify, and produce one CSV row with 23 fixed columns. `metrics_csv_header` and `metrics_csv_row` are shared so rows from different runs concatenate. Nothing expands a family of scenarios, runs many in one process, varies a scenario along named axes, or analyses the rows. The control components (prediction, fallback, separation) cannot be turned off from a scenario file, so the ablations cannot be expressed yet.

## Decisions

### The evaluation matrix is six axes, and the cell count is 432

The plan's axes are loss level, weather, jamming, drone count, faults, and scene. The product is 4 loss levels (0, 10, 30, 50 percent) x 3 weather (clear, fog, rain) x 3 jamming (none, constant, pulsed) x 3 drone counts (5, 10, 20) x 2 faults (none, one drone fails) x 2 scenes (field, hills) = 432 cells. At 5 seeds that is 2,160 runs, and the final run raises the seed count, which is one configuration value.

Random loss against burst loss at equal mean is a comparison, not another full axis: crossing it with everything else would make 864 cells for a question about one factor. It is run as a separate experiment over the loss levels alone, on the baseline scene. The baselines and the ablations are likewise separate experiments on a smaller matrix, because comparing methods does not need every environmental combination. All of them use the same machinery; they differ in the configuration file.

### An experiment is a file, and an axis level is a fragment merged over the base scenario

An experiment file is strict, versioned YAML like a scenario file (unknown keys are errors, errors carry a path and a line). It names a base scenario file, a first seed and a seed count, and a list of axes. An axis has a name and a list of levels; a level has a label and a `set` mapping, which may be empty for "no change". The `set` mapping is written in the scenario format itself and is merged over the base: maps merge key by key, and any other value (a number, a list such as the jammers or the faults) replaces the base's.

Two designs were considered. A fixed vocabulary of axes, each a typed patch written in code, makes every axis a code change and duplicates what the scenario format already says. Merging scenario fragments needs no new code per axis. Its risk is that a typo selects a key that does not exist or a value of the wrong type, and the answer is that every cell's merged text goes through the ordinary scenario loader at expansion, before anything runs, so the strict rules still apply and the error names the cell. The experiment owns the seed, the name, and the version: a fragment that sets them is an error. Two axes that write the same value are an error too, because the outcome would depend on their order. The values that give "fog" or "pulsed" their meaning are in the experiment file, in version control, not in the code.

### Cells are enumerated in a fixed order, and seeds are shared

The cell list is the product of the axes in the order the file lists them, the last axis varying fastest. A cell has an index, a label per axis, and a name made from the labels. Replicate `r` of every cell uses seed `first_seed + r`. Sharing the seed across cells makes comparisons between cells paired (the same terrain, start jitter, and noise draws where the cell allows it), which is what ablations and baselines need to isolate an effect. The consequence is that the replicates of one cell are not independent of the corresponding replicates of another cell, and the analysis uses pairing where it compares cells and treats replicates within a cell as independent draws.

The expansion is a pure function of the file. A test fixes the planned matrix at 432 cells and checks the enumeration order, the names, and that no two cells are identical.

### The batch driver runs one scenario per task, and the output order does not depend on timing

Each (cell, seed) is a task: build the scenario, run it on one thread with the deterministic channel, compute the metrics from the in-memory log, and keep one result row. Tasks run on a pool of worker threads sized by `--jobs`. Workers take tasks from a shared index in order, but a result is stored at its task's index and the table is written in index order at the end, so the output bytes are the same for one worker and for many. Nothing in a task reads the clock or shares mutable state with another task, and the simulation inside a task is already independent of thread count, so a test can run the same small experiment with 1 and with 6 jobs and compare the files byte for byte.

A task that fails (a scenario that is invalid after patching, a log that cannot be written) does not stop the batch and does not disappear: its row has a status column with the reason and empty metric cells, and the command exits non-zero. A missing row would silently bias a table; a failed row cannot.

The log of a task is verified by default on a sampled basis (every Nth task re-executed and compared), because verifying all 2,160 would double the cost for little added assurance once the determinism tests pass. The sampling is by task index, so it is reproducible, and `--verify-all` is available. Logs are not kept unless `--keep-logs DIR` is given, because a thousand logs are hundreds of megabytes and the table is what is analysed.

### The result table has the metrics columns plus the cell

One row per task: the experiment name, the cell index, one column per axis holding the level label, the seed, a status, and then the 23 metric columns exactly as `metrics_csv_row` writes them. Reusing that function means a single run and a batch cannot report differently. The header is written once. A metadata file next to the table records the experiment text, the build's version string, the compiler and platform, and the number of tasks; the table itself contains no timing or host information, which keeps it reproducible.

### Ablations disable exactly one component

Two switches are added to the control stack, both on by default: the fallback state machine (off means the drone always behaves as in the nominal state; the machine still runs and reports its state, so the log shows when it would have acted, and a commanded hold still works because it is not a fallback), and the separation rule (off means no repulsion term). The third ablation, prediction, needs no switch: it is the hold-last predictor, which treats a neighbor as stationary at its last report, and the scenario format already selects it. It is the same configuration as the hold-last baseline, and the documentation says so once instead of presenting it as two results.

The scenario format reads the switches as `control.fallback` and `control.separation` (true or false). The canonical text writes a switch only when it is off, so the header of every existing log, and so every existing hash, is unchanged. Tests check that a switch at its default leaves the log bytes unchanged, that each switch off changes the run, and that the changed log still verifies, because the switch is part of the scenario in the header. Experiment files `ablation.yaml` and `baselines.yaml` hold the two comparisons; a test checks that each ablation variant differs from the full controller in that one setting and nowhere else.

### Analysis is Python reading CSV, with pinned dependencies and tests

The analysis is a small package under `analysis/` using numpy, scipy, and matplotlib, with exact versions in a requirements file and pytest for tests. It does not run the simulation. Reading a table, grouping by cell, and computing a summary per metric are separate functions, each tested on inputs whose answers are known by hand.

The interval for a cell's mean over seeds is a two-sided 95 percent Student t interval, which is what a handful of seeds supports; with 5 seeds the interval is wide, and the output says how many seeds it used. The median and the minimum and maximum are reported alongside, because several metrics (collision episodes, recovery time) are skewed or censored. Recovery time is reported with the count of censored runs, never averaged over only the runs that recovered without saying so. Comparisons between two cells use the paired differences over shared seeds. Nothing is tested for significance across the whole matrix; there are hundreds of comparisons and the project does not claim a finding from one of them.

### Figures are scripts that read the summary and write files

Each figure is a function from a summary table to a file, with fixed sizes, fonts, colours, and a fixed matplotlib backend, so they can be regenerated from one command. A test runs them on a small fixture table and checks that the expected files exist and are non-empty images of the stated size. No test compares pixels. Axis labels carry units and the number of seeds. A figure does not smooth between points that were not measured.

### The fog check compares the model with a published curve, and says what it is

The weather model has a visibility parameter that sets how quickly the LiDAR return probability falls with range. One published description of LiDAR detection range against visibility is chosen and cited in the documentation with its source and the points taken from it, and the model's output at those visibilities is compared with it within a stated tolerance. If the model does not match, the model's constant or the claim changes, and the outcome is written down either way. This is one sourced fit, not a validation of the weather model, and the documentation says that the rain and the jammer models remain assumptions.

## Components and contracts

### Experiment description and expansion (commit 57)

`core/experiment`: `Experiment` (name, base scenario text, first seed, seed count, axes), `load_experiment` with the strict-YAML rules and a caller-supplied file reader (so loading and expansion do no file access of their own), and `expand(experiment)` returning the ordered cells, each with its index, level indices, name, and `Scenario`. Errors for a duplicate axis or label, an empty axis, a bad name or label, a fragment that sets the seed, name, or version or is not a mapping, two axes writing the same value, a fragment that makes a cell's scenario invalid, a seed count of zero, a base that cannot be read, and more than 100,000 cells. The `experiments/` directory holds the matrix of 432 cells and the smaller ones for the loss model, baselines, and ablations.

### Batch driver (commit 58)

`core/experiment`: `run_batch(experiment, jobs, options)` returning the rows in task order, and the `swarm_batch EXPERIMENT --out FILE --jobs N` tool (separate from `swarm_sim`, which stays a single-run program). `--keep-logs`, `--verify-all`, `--seeds N` to override the count for a quick run. Progress goes to standard error and nothing in it enters the table.

### Ablations (commit 59)

The two control switches (fallback and separation), their scenario keys, loader and canonical text support, brain tests per switch, and the ablation and baseline experiment files. The scenario format version stays 1 because the new keys are optional.

### Analysis (commit 60)

`analysis/swarmsim_analysis`: `load_table`, `summarize`, `paired_difference`, with pytest tests for known intervals (a sample with a hand-computed mean and t interval), a censored column, a single-seed group (interval undefined, reported as such), and malformed tables.

### Figures (commit 61)

`analysis/swarmsim_analysis/figures.py` and a `make_figures.py` entry point: formation error and delivery ratio against loss, recovery time against jamming, and the ablation comparison. Tests on a fixture table.

### Fog fit (commit 62)

A test (`tests/unit/test_fog_fit.cpp`) and a section in MODELS.md. The source is Rasshofer et al. (2011), whose Fig. 7 caption gives `alpha = 3 / visibility` for fog measured at 50 to 205 m with a 905 nm sensor. The paper reports qualitative agreement and no numeric error, so the check is not a tolerance on a curve: it shows that the model reproduces the published relation exactly with a fog constant of 3.0, and that the default 3.912 differs from it by a factor of 1.304, which is the difference between the 2 percent and 5 percent visibility conventions. The commit adds `weather.fog_constant` (default unchanged, written to the canonical text only when it differs) and `experiments/fog_convention.yaml`, which shows where the position fix is lost under each constant. The outcome: the default model stands, the difference is documented, and the setting exists for a sensitivity check.

## Tests

Counts and order of the matrix; each unknown or empty or duplicate case rejected with a path; the same experiment gives the same table with 1 and 6 jobs; a failing task becomes a row with a status and a non-zero exit; the verification sample is reproducible; each ablation switch changes only its component; the default switches leave the existing hashes and log bytes unchanged; the analysis on known data; the figures on a fixture; the fog check.

## Documentation produced

MODELS.md sections for the experiment format, the driver, the ablations, the statistics, and the fog fit; an ADR for the merged-fragment axes and the shared seeds; DETERMINISM.md on why batch output is independent of job count; DEVELOPMENT.md on setting up the Python environment and regenerating the tables and figures; PLAN.md change log and README status at the end of the phase.

## Risks

| Risk | Handling |
|---|---|
| The full matrix is too slow to run often | Measure one cell first and report it; `--seeds` and the smaller experiments cover routine use; the final run is one command |
| Batch output depends on scheduling | Results stored by task index and written in order; a test compares 1 and 6 jobs byte for byte |
| A failed run silently biases the table | A failed task is a row with a status, and the exit code is non-zero |
| Few seeds give intervals that look more certain than they are | The seed count is in every output; t intervals; the claim is limited to the differences the intervals support |
| Shared seeds hide variance between cells | Stated; comparisons use pairing and within-cell statistics use only the cell's own replicates |
| Ablation switches change normal behaviour | Defaults leave behaviour unchanged, with a test that existing hashes and log bytes are the same |
| Python environment differences change figures | Pinned requirements, a fixed backend and styling, tests check files and sizes and not pixels |
| The fog curve does not match | The outcome is recorded either way; the model or the claim changes |
| Tuning the matrix toward a good-looking result | Level values and metrics are fixed in the experiment file before the final run, in version control |

## Exit criteria

- All six commits made, each green locally and in CI.
- The planned matrix expands to 432 cells, and a small batch gives byte-identical tables for different job counts.
- The analysis tests pass on known inputs, and the figures regenerate from one command on a fixture.
- The fog check is written up with its source and outcome.
- MODELS.md, the decision record, DETERMINISM.md, and DEVELOPMENT.md updated; PLAN.md change log and README status updated.

## Outcome

Phase 7 was completed in six commits (57 to 62). One command, `swarm_batch`, expands an experiment file and runs every cell for every seed on a pool of workers; `python -m swarmsim_analysis` turns the table into summaries and figures.

What was verified, by tests: the planned matrix expands to 432 cells; the results table is byte-identical for 1, 2, 3, 6, and 40 jobs, in the unit tests and through the command line; a failed run is a row with a status; each ablation variant differs from the full controller in one setting only; the statistics match hand-computed Student t intervals; figures are valid PNGs of the fixed size and the same table gives the same bytes; and the fog model reproduces a published relation when its constant is set to the published value.

Changes from the design above:

- Axis levels are fragments in the scenario format merged over the base, not typed patches in code (see the decision above).
- Two ablation switches were added (fallback and separation). Prediction is the hold-last predictor and needs none.
- `swarm_batch` is a separate tool and not a `swarm_sim` subcommand.
- `weather.fog_constant` was added so that the fog relation can be set to the published value. The default is unchanged.
- An axis cannot be named like a column of the results table. A first smoke experiment had an axis named `drones`, which made two columns of that name; the loader now rejects it.

Findings:

- At 45 m altitude the LiDAR position fix finds no ground returns, the drones enter safe hold, and the settled formation error in a 60 s run is over 100 m. The experiments fly at 20 m. This is a limit of the sensor model that the evaluation does not exercise.
- The default fog constant (3.912, 2 percent contrast) is 1.304 times the value in the one published relation checked (3, which matches the 5 percent convention). In the base scenario the position fix is lost between 200 m and 150 m visibility by default, and between 150 m and 120 m with the published constant.
- In a first three-seed look at the ablation experiment (a pulsed jammer, one drone failing), removing prediction raised the settled error from about 0.75 m to about 5.3 m, removing separation lowered the closest approach from about 4 m to about 1.2 m, and removing the fallback changed nothing measurable. These are preliminary; the five-seed run is in the release phase.

Open items:

- The full runs (matrix, ablation, baselines at five seeds, and the 30-seed rerun) and the results table in the README belong to Phase 9.
- Rain constants are still placeholders; the fog check covers one sensor type and one fog type.
- The statistics use t intervals that assume roughly normal values, which several metrics are not; see MODELS.md.
- No continuous-integration run has exercised the new Python job yet at the time of writing.
