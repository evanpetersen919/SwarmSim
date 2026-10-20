# Phase 8: Unreal viewer

Status: planned. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [decision 0002](../decisions/0002-unreal-integration.md), [decision 0003](../decisions/0003-scene-data-in-format.md), [phase 5](phase-05-scenarios-and-metrics.md).

## Goal

A recorded run plays back in Unreal Engine 5.4 as a presentation: the drones fly the logged paths with spinning rotors over generated terrain with obstacles, trees, and grass that move in the wind, with playback controls, several cameras, and the swarm's state shown on the drones. The scene is built entirely from code from the run's log and its scene file. Nothing is placed by hand in the editor, and the viewer shows what was recorded; it does not simulate.

## Non-goals

Running the simulation inside Unreal, packaging a shipped game, networking, a physics simulation in the viewer, VR, and editing a run. The viewer does not recompute anything that the log records, and it does not compute the simulation's state hash.

## What exists and what is missing

The plugin module `SwarmSim` compiles the `core/format` sources and nothing else. The host project `UnrealHost` loads it. Both build from the command line with Unreal Engine 5.4.4 (see DEVELOPMENT.md); the module builds without unity builds. There is no code in the plugin yet besides the empty module.

The log holds, for every tick, the state hash and a snapshot: each drone's true position, velocity, alive flag, and fallback state, and the outcomes of the messages sent that tick. The drone states are stored on the ticks that the scenario's snapshot stride selects and on the last tick; the other ticks carry events only. The scene (terrain heightfield and obstacles) is not in the log. The scenario, in canonical text, is in the log header, and the scene is a pure function of it. The log has no attitude (no orientation of a drone), no wind, and no rotor speed.

## Decisions

### The viewer reads two files: the log and a scene file

The plugin cannot generate the scene, because the generator is in `core/sim`, which uses Eigen, and Unreal code never includes Eigen. So a command, `swarm_sim export-scene LOG --out FILE`, reads the scenario from the log's header, builds the world, and writes the scene in the existing serialized form (`serialize_scene`, with a content hash). Deriving the scene from the log guarantees that the pair match. The viewer opens the log, opens the scene file next to it, and refuses a scene whose grid or obstacle data is invalid. Embedding the scene in the log was considered and rejected: it would bump the log version and enlarge every log by the size of the terrain for the benefit of one tool.

### A replay reader in `core/format`, tested in the core

The plugin needs the log as arrays of frames, not as a stream. A reader in `core/format` (`Replay`) opens a log from memory, checks it with the existing `LogReader` (magic, version, record order, trailer count and hash), decodes every snapshot, and exposes the header, the frames, and the trailer. Errors are status codes, nothing throws, and it has no Eigen or yaml-cpp, because it is compiled into the plugin. Because the same sources are in the core build, its tests run in CI, which cannot build Unreal. The reader also answers the one question the viewer asks: where is drone `i` at simulation time `t`.

### Motion between logged frames is interpolated, and orientation is derived and cosmetic

Drone states exist on the stride ticks, and a viewer frame rarely falls on one. The position between two logged frames is a cubic Hermite curve through the two positions with the two velocities, so a drone does not move in straight segments with corners, and a frame on a logged tick is exactly the logged position. Alive and fallback state come from the earlier frame. The interpolation lives in `core/format` and is tested for exactness at the endpoints, continuity of velocity at the joins, and no overshoot for a straight constant-velocity flight.

The log has no attitude, so the viewer derives one for display: heading from the horizontal velocity, and a tilt from the acceleration (the change of velocity between frames), limited to a plausible angle. It is cosmetic, the documentation says so, and nothing measured depends on it. Rotor speed is a fixed spin rate while the drone is alive and zero when it has failed.

### Assets are referenced by soft path, with a procedural fallback for each

For the presentation the drone, trees, grass, and terrain material come from asset packs the author owns. They are not in the repository: free and marketplace packs usually cannot be redistributed, and they are large (`.uasset` and `.umap` go through Git LFS if any are ever committed). A settings object in the plugin holds a soft object path for each: the drone mesh, an optional rotor mesh, the terrain material, tree meshes, grass meshes, a building or rock mesh for box obstacles, and a wind parameter collection. When a path is empty or the asset is missing, the viewer builds a procedural stand-in from engine primitives (a quadcopter from cubes and cylinders, cone and cylinder trees, a flat-coloured terrain), so the plugin works, builds, and passes its automated checks with no assets installed. Which packs were used, and their licences, are recorded in the documentation, because a published demo has to respect them.

### Everything in the scene comes from code

- **Terrain.** An Unreal Landscape actor built from the heightfield, one Landscape vertex per heightfield sample, with the material and ground layers of the terrain asset pack (see the decision below). The simulation's meters and right-handed frame are converted to Unreal's centimeters and left-handed frame in one place, by negating y. A grid that is not a size a Landscape allows is padded by repeating its edge heights, outside the simulation's world. Ground layers (grass, rock, mud, two forest floors) are weighted from slope, height, and seeded noise, so the same log gives the same ground.
- **Obstacles.** A cylinder obstacle becomes a tree and a box obstacle becomes a rock. A tree is the mesh whose height is closest to the cylinder's, scaled uniformly to that height (limited to 0.4 to 3 times), standing on the cylinder's base, turned to a yaw that depends on its position and the seed. A rock is scaled in each axis until the mesh's bounds are exactly the box, and it is not turned, because the box is aligned with the axes; a box much taller than it is wide is a stack of rocks that together fill it, because one stretched rock looks like a pillar (found in the first render). Each mesh gets one instanced component. The count equals the scene's obstacle count. The simulation's cylinders are thin (up to 1.5 m in radius) and a pine's crown is wider, so a pine looks bulkier than the cylinder the sensor sees; the viewer does not change that.
- **Grass and bushes.** The asset pack's landscape material scatters them itself, as landscape grass types, on the parts of the terrain painted as grass, within a few tens of meters of the camera. A close-up render with the pack confirmed this, so the viewer adds no scatter code of its own; it only paints the ground layers. (An earlier design scattered them from code with a seeded generator. That is only needed if a terrain asset has no such grass.)
- **Wind and snow.** The trees, grass, and bushes move because the pack's materials read a material parameter collection (`global_wind`: `Wind_Intensity`, `Wind_X_Speed`, `Wind_Y_Speed`, `Wind_Variation_Tiling`). The viewer sets those scalars from code from its settings, and `ASwarmScene::SetWind` changes them. The same pack's `master_snow_adjustment` collection has `Snow_Coverage`, which `ASwarmScene::SetSnowCoverage` sets from 0 (summer) to 1. A collection that is not installed is skipped, and assets whose materials do not read these parameters stand still.
- **Light and sky.** A sun, sky atmosphere, sky light, height fog, and exposure, created by the viewer. The fog density follows the scenario's visibility, so a foggy scenario looks foggy (commit 67).

### Playback is driven by the log, and the hash on screen is the log's

A playback actor holds the current simulation tick as a real number, advanced by the frame's time and a speed from 0.25 to 8 times, with play, pause, scrub, and restart. The drones, camera, and heads-up display read from it. The heads-up display shows the tick, the simulation time, the speed, the number of drones alive, and the log's state hash for the current record. At the last tick that hash is the log's trailer hash, which equals the hash `swarm_sim` prints when it runs or verifies the same log. That is what "the playback hash matches the log at the final tick" means: the viewer shows the recorded value faithfully. It is not a re-computation, and the documentation says so.

### Cameras

A free camera, an orbit around the swarm's centroid, a follow camera on a chosen drone, and a fixed cinematic path that crosses the scene, switched by key and by the interface. The camera never changes anything that is shown; only where it looks from.

### The drone state is shown on the drone

Each drone carries a coloured indicator for its fallback state (nominal, degraded comms, degraded navigation, safe hold) and goes dark when it has failed. Message events can be drawn as short arcs between drones, colour-coded by outcome, with a limit on how many are drawn per frame; this and the weather and jamming visuals are the optional last commit.

### The automated checks run Unreal without a window

CI cannot build Unreal, but a headless editor can run automation tests on this machine with no rendering. The plugin's checks are written as Unreal automation tests and run from the command line: the terrain vertices at sampled positions equal the core's height queries, the number of spawned obstacles and drones equals the scene's and the log's, the stand-in meshes are used when no assets are set, and playback at the last tick reports the log's final hash. The manual checks are what an automated test cannot see: that it looks right, that the grass and trees move, and that the camera feels right.

## Components and contracts

### Replay reader and scene export (commit 63)

`core/format`: `Replay` with `load(data, size)` returning a status, the header, the frames in record order (tick, hash, and the snapshot, with the agents empty on ticks that carry no drone state), the trailer, and `sample(time, agent)` with the Hermite rule. Statuses for every way a log can be bad, passed through from `LogReader` and `decode_snapshot`, and a status for a log that holds no drone state at all. `swarm_sim export-scene LOG --out FILE`. Core tests for each rule above and a round trip through the file. The plugin builds with the new sources.

### Load and terrain (commit 64)

The plugin settings object (soft paths to the terrain material and the ground layers, with the pack's assets as defaults), scene file loading, the coordinate conversion, and the Landscape builder (editor only, because `ALandscape::Import` exists only in the editor). Terrain is a Landscape and not a procedural mesh because the chosen asset pack is built for Landscapes: its materials use landscape layers, and its grass and bushes are landscape grass types that Unreal scatters, with wind, on its own. The cost is that a Landscape triangulates its quads its own way, so inside a cell its surface differs slightly from the simulation's `(a, b, c)`, `(a, c, d)` split; at the grid points the heights are equal. A procedural mesh with the exact triangulation remains the fallback for a build without the pack and for the exact-geometry claim.

Automation tests, run without a window: the layout choice, the height encoding and the orientation rule, the ground weights, the Landscape against a synthetic scene, and the Landscape with the pack's assets when they are installed. The Landscape's stored heights are compared at every vertex through the editor's data interface, and its placement in the world through the engine's own height query at the interior samples, with a spike placed at a known sample and checked at numbers written out in the test, not through the conversion helpers, so a mirrored or transposed terrain fails.

### Spawn from code (commit 65)

`ASwarmScene`, an actor that builds the static scene from a scene file: the terrain, the trees and rocks at the simulation's obstacles, and the wind and snow. The obstacle placement is plain C++ with its own tests; the actor loads the meshes named in the settings and falls back to the engine's cone and cube when they are missing. Drones are not part of this commit (they come with playback, commit 66), and the drone model is chosen separately. Automation tests: the fit of every tree and rock against numbers written out in the test (base, height, and the bounds of a rock against its box, with the simulation's y mirrored), determinism, a seed that turns trees but does not move them, the scale limits, stand-ins used with nothing installed, and, when the pack is installed, the scene built from a file with counts equal to the scene's obstacles and the wind and snow values read back from the collections.

### Playback, cameras, and display (commit 66)

The playback actor, interpolation, derived attitude, rotor spin, the cameras, the heads-up display, and the state indicators. Automation test: playback to the last tick shows the trailer hash.

### Atmosphere and overlays (commit 67, optional)

Sun and sky, fog from the scenario's visibility, jamming zones, rain, and message arcs. Reading the scenario's weather and jammers from the log header needs a reader for the canonical text in the plugin (no yaml-cpp), so that commit either adds a small reader of the canonical text to `core/format` or has the export step also write a viewer manifest; the choice is made when the commit is designed.

## Tests and checks

Core tests (CI): the replay reader on good and corrupt logs, every truncation, a log with no drone state, frames in order, the trailer hash, the Hermite endpoints and continuity, and the scene export round trip. Unreal automation tests (local): as above. Manual: the terrain looks like the heightfield, grass and trees sway, the drones fly the logged paths with spinning rotors and tilt, the cameras work, the state colours change when a drone degrades or fails, and the on-screen hash at the end equals the one the command line prints.

## Documentation produced

MODELS.md sections for the replay reader and the interpolation, the scene export, and the viewer's conventions (units, axes, derived attitude); an ADR for the two-file replay and the soft-path assets; DEVELOPMENT.md on running the viewer and the automation tests and on which asset packs and settings the author used; PLAN.md change log and README status at the end of the phase.

## Risks

| Risk | Handling |
|---|---|
| The viewer shows something that is not what was recorded | It reads the log and the scene file only, interpolates only between logged frames with exact endpoints, and shows the log's hash |
| Assets are unavailable or not redistributable | Soft paths with procedural stand-ins; none committed; the packs and licences are documented |
| The scene file and the log do not match | The scene is derived from the log's own header |
| Unity builds or exceptions break the plugin build | `bUseUnity = false`; the shared sources are exception-free and built in CI as C++20 |
| Coordinate mistakes (units, handedness, axes) | One conversion function, tested; the terrain test compares vertices with the core |
| Large scenes are slow | Limits on instances and on the heightfield size; instanced meshes; a cap on message arcs |
| Foliage does not move | The wind parameters are set from code; the documentation states what an asset needs; the stand-ins are static and say so |
| Headless Unreal tests are fragile | They are local, not in CI, and test data and counts, not rendering |
| The log has no attitude | Derived from velocity and acceleration, limited, and labelled cosmetic |

## Exit criteria

- All five commits made, each green locally and in CI for the core part, and the plugin builds from the command line.
- A recorded run plays back from its log and scene file with no editor placement, in a scene with moving foliage when the asset packs are installed and with stand-ins when they are not.
- The automation tests pass locally and the manual checks are done by the author.
- The hash on screen at the last tick equals the one the command line prints.
- MODELS.md, the decision record, and DEVELOPMENT.md written; PLAN.md change log and README status updated.
