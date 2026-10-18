# 0007. Strict, versioned scenario files with project-parsed scalars

Date: 2026-10-18. Status: accepted.

## Context

A run is meant to be fully described by a scenario file and a seed, and the results in the plan rest on those files being read exactly as written. A configuration file fails quietly in two ways. A misspelled key leaves a default in place and the run answers a different question than the one asked. And the numbers read out of the file depend on how a library converts text, which differs between libraries and versions.

## Decision

Scenario files are YAML with a top-level `version`, read by yaml-cpp 0.8.0 for document structure only.

- Unknown keys are errors, and so are missing required keys. A file is rejected as a whole; there are no warnings.
- The loader collects every problem and returns them with a dotted path and a line number, in file order, so a file is fixed in one pass.
- Every scalar is taken as text and converted by the project: decimal notation only, the whole text consumed, finite results only. `inf` is accepted for a jammer radius and nowhere else. Booleans are `true` or `false`. This excludes forms such as `.inf`, `nan`, hexadecimal, and `yes` that a general YAML library would convert.
- Times are written in seconds and converted to ticks. A time that is not a whole number of ticks is an error, so nothing is rounded without being said.
- Values are checked against the existing validators of the model structs (terrain, obstacles, weather, mission, channel, LiDAR), so a loaded scenario cannot construct an invalid simulation.
- The thread count is not part of a scenario, because it must never change results.
- yaml-cpp is fetched at a pinned version and checked against a SHA-256 of the archive, built as a static library without tests or tools, and linked only into the scenario library. Its headers are system includes for the project's warnings. `core/format`, which is compiled into the Unreal plugin, does not use it, and the invariants script checks that.

## Alternatives considered

- Let yaml-cpp convert scalars: shorter code, but behavior the results then depend on, and silent acceptance of forms the project does not want.
- JSON or a key=value text format: no new dependency, but harder to write by hand and to comment, and the plan names YAML.
- Lenient loading with defaults and warnings: friendlier, and exactly the failure the first paragraph describes.

## Consequences

- Adding a key is a schema change: files using it need a version that knows it, and old builds reject it as unknown, which is the intent.
- The loader is longer than a direct mapping of the file onto a struct. The cost is paid once and tested.
- Error messages are part of the interface and are tested for their path, line, and wording.
