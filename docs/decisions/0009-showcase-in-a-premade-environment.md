# 0009. Benchmarks on generated scenes, the showcase in a premade environment, with the simulation run on exported geometry

Date: 2026-10-20. Status: accepted.

## Context

The viewer built from the pack of nature assets does not look like a recent game, and no amount of lighting changes that, because the vegetation and ground are modest assets. Epic's Electric Dreams Environment sample (a 4 km by 4 km World Partition landscape with a Nanite forest, rock outcrops, and a river area, built with the PCG framework) does look photographic, and it runs on this machine as a standalone game in UE 5.4 (about 9 GB of the game's own RAM on a 32 GB machine; the full editor with the whole level streaming does not run comfortably, because it reached 28 GB and stalled).

Two properties of the project pull against using a premade level:

- **Reproducibility.** The results table in the README must be something anyone can regenerate from files in the repository. A level from Fab or an Epic sample cannot be redistributed, and it is a hand-made map.
- **Truth.** The viewer must show what the simulation used. If the simulation ran on generated hills and the viewer showed a forest, drones would visibly fly through trees.

## Decision

1. **The benchmark stays on generated scenes.** Every number in the README comes from experiment files and scenarios in the repository. The viewer and the premade level are not part of it, and the simulation never renders anything, so the look of a scene does not change a benchmark number.
2. **The showcase runs in a premade environment, and the simulation runs on that environment's geometry.** A scenario can name a scene file instead of generating terrain. The scene file is exported from the running level (its terrain heights and its trees and rocks as cylinders and boxes, the only shapes the sensor model knows). The simulation then senses and avoids the obstacles the viewer shows. The scene file's content hash goes into the scenario in the log header, so a log can be verified against the file it names. The scene file is derived from licensed content and is kept out of the repository, like the content itself.
3. **The viewer needs no change to the premade level.** The plugin adds a world subsystem that, when a replay is requested on the command line, spawns the playback actor in whatever level is loaded. Nothing is placed in the level by hand and the level is not modified. A single anchor (the level's x and y, in centimeters, of the simulation's origin) ties the two frames together; heights are absolute in both.
4. **Standalone game mode, not the editor,** for anything that streams a large level, because of memory.

## Consequences

- A showcase run is reproducible only for someone who has the same premade level and the exported scene file. The documentation says so, and it names the level and the engine version.
- The exporter is a new piece of code (it reads a running level) and a new kind of scenario (scene from a file). Both are tested: the scene-from-file path in the core, and the exporter against a synthetic world in an Unreal automation test.
- The licence of the premade level is not stated in the project and was not confirmed from the Fab listing. Screenshots and video of it are likely allowed and its files are not redistributable. The repository holds none of it.
- Trees as cylinders and rocks as boxes are an approximation of the real foliage. A pine's crown is wider than its trunk and a dense canopy is more than a set of cylinders. The sensor model sees only what the exporter gives it, and the documentation says what the approximation leaves out.
- The drone model is a separate choice and does not depend on this one.
