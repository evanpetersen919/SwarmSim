# Phase 2: Vehicles and sensing

Status: planned. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md), [phase 1](phase-01-deterministic-runtime.md).

## Goal

Drones that move under limits, sense a generated scene with an IMU and a LiDAR, and communicate through a channel that jamming zones and weather can degrade. All of it deterministic and safe to run on the lockstep pool.

No estimation, control, formations, scenario files, or Unreal in this phase. Sensors are built and tested standalone, then proven together on the lockstep loop in the last commit.

## Non-goals

Attitude dynamics (roll and pitch), motion distortion of LiDAR scans, camera models, GPS, scene file import, urban and cave scenes, and any estimator that consumes sensor output.

## Conventions

Recorded here and repeated in [MODELS.md](../MODELS.md) once the first model lands.

- SI units: meters, seconds, radians. Positions are world coordinates, right-handed, +z up, +x forward at yaw 0, yaw counter-clockwise about +z.
- Gravity is `(0, 0, -9.81)`. Dynamics ignore it (thrust compensates); the IMU model accounts for it.
- The Unreal plugin converts to Unreal's left-handed centimeter coordinates at its boundary (Phase 8). The core never uses Unreal units.

## Decisions made now because they are expensive to change later

### Scene data lives in `core/format`, scene logic in `core/sim`

The Unreal plugin compiles only `core/format`. It must be able to rebuild the exact terrain and obstacles the LiDAR saw, without the generator. So:

- `core/format`: `SceneData` (plain heightfield and obstacle lists), its binary serialization, validation, and a content hash. No dependencies, no exceptions.
- `core/sim`: terrain and obstacle generators that produce `SceneData`, and `Scene`, which wraps a `SceneData` with ray-casting acceleration.

Embedding `SceneData` in the run log is deferred to Phase 5 (scenario runner) and will bump the log version then. Phase 2 only guarantees the data and serializer exist and are tested.

### Terrain is a triangle mesh with a fixed diagonal

Terrain is a regular grid of samples. Each cell is split into two triangles along the diagonal from its low corner to its high corner:

```
cell (i, j), corners a=(i,j) b=(i+1,j) c=(i+1,j+1) d=(i,j+1)
triangles: (a, b, c) and (a, c, d)
```

Height queries, ray casts, and the Unreal mesh all use exactly these triangles. Bilinear interpolation is not used anywhere, because a bilinear surface and a triangulated mesh disagree inside a cell, and the viewer would then show terrain the LiDAR did not see.

### Sensors use their own RNG streams

New stream ids in `swarm/streams.hpp`:

| Stream | Id | Used for |
|---|---|---|
| Channel jam | `0x103` | Probabilistic jamming drops |
| Scene | `0x104` | Obstacle placement |
| IMU of drone `i` | `0x2000 + i` | IMU noise and bias |
| LiDAR of drone `i` | `0x3000 + i` | Range noise and dropouts |

Each base supports ids below 4096. `Simulation` rejects more than 4096 agents so streams cannot collide. Terrain noise uses an integer hash, not an RNG stream, so it needs no stream id.

### Fixed random draws per event

A sensor draws the same number of random values per sample or per ray regardless of what it sees. Changing the weather or the scene therefore never shifts the noise applied to unrelated rays. Tests assert this.

### Const scene queries are thread-safe

`Scene::raycast` is `const`, uses no caches, and has no mutable members. Many drones query one shared scene from pool threads. The last commit tests this under the pool, and the ThreadSanitizer CI job covers it.

### Float determinism boundaries

Guaranteed identical across compilers and platforms: anything built from integer hashing and basic IEEE arithmetic (terrain noise, ray-triangle tests). The terrain test pins a golden hash of a small generated scene, so a platform or flag change that breaks this fails loudly.

Not guaranteed across platforms: anything using `sin`, `cos`, `log`, or `exp` (LiDAR beam directions, Box-Muller noise, intensity attenuation), because libm results can differ in the last bit. The determinism guarantee for those is the one in [DETERMINISM.md](../DETERMINISM.md): same binary and platform.

## Components and contracts

### Vec3 and dynamics (commit 10)

- `Vec3` of doubles with the usual operators, `dot`, `cross`, `norm`, `normalized` (zero stays zero), `clamp_norm`.
- `DroneState { pos, vel, accel, yaw, yaw_rate }`. `accel` is the acceleration actually applied last step.
- `DroneParams { max_speed, max_accel, max_yaw_rate }`.
- `DroneCommand { accel, yaw_rate }`.
- `step(state, command, params, dt)`: clamp the commanded acceleration to `max_accel`; `v' = v + a*dt`; clamp `|v'|` to `max_speed` keeping direction; store `accel = (v' - v)/dt`; `p' = p + v'*dt` (semi-implicit Euler); clamp and integrate yaw, wrapped to `(-pi, pi]`.
- Invalid `dt` (not positive) is an error.

### IMU (commit 11)

Orientation-free model: a world-frame accelerometer and a yaw-rate gyro. It reports specific force `f = a - g` (so a hovering drone reads `+9.81` on z) and yaw rate, each with bias and noise.

- Parameters: accelerometer and gyro noise density, accelerometer and gyro bias random-walk, initial bias sigma. Discrete-time per-sample sigmas: noise `density / sqrt(dt)`, bias step `walk * sqrt(dt)`.
- Every sample draws the same normals (bias steps for 3 axes plus gyro, noise for 3 axes plus gyro) whatever the parameters are.
- Limitation, stated in MODELS.md: no attitude, so no tilt-dependent gravity coupling.

### Scene data (commit 12, `core/format`)

- `Heightfield`: `cells_x`, `cells_y`, `origin_x`, `origin_y`, `cell_size`, and `(cells_x+1)*(cells_y+1)` heights, row-major with x varying fastest.
- `Obstacle`: kind (box or vertical cylinder), center x and y, `base_z`, `height`, and size (`half_x`, `half_y` for a box; `radius` stored in `half_x` for a cylinder).
- `SceneData { Heightfield, std::vector<Obstacle> }`.
- Binary layout with magic `SWSC`, version, little-endian, same status-code error style as the log. Limits: at most 4096 cells per axis and 1,000,000 obstacles, checked before allocating.
- `validate`, `scene_hash` (FNV-1a over all fields), `serialize`, `parse`.

### Terrain generator (commit 13)

- `TerrainConfig`: extent, cell size, type (`field` or `hills`), amplitude, feature scale, octaves, persistence.
- Hills use 2D gradient noise on an integer lattice. Gradients come from a fixed table of eight directions indexed by a splitmix64 hash of (seed, lattice x, lattice y), so no libm and no RNG stream is involved. Octaves sum with halving amplitude and doubling frequency and are normalized so `|height| <= amplitude`.
- `field` is exactly flat at height 0.
- `height_at(scene, x, y)` evaluates the triangulated surface exactly and is shared by obstacle placement and tests.
- Grid is centered on the origin. Extent is rounded up to a whole number of cells.

### Obstacle placement (commit 14)

- `ObstacleConfig`: density per square meter, minimum edge-to-edge spacing, box and cylinder size ranges, cylinder fraction, spawn center and clear radius.
- Seeded rejection sampling on stream `0x104`. Each attempt draws the same values whether or not it is accepted. Attempts per obstacle are capped, so a saturated scene places fewer obstacles and terminates.
- Spacing and spawn clearance are tested against each obstacle's bounding circle (a conservative rule, stated in the docs).
- `base_z` is the lowest terrain height under the footprint minus a small embed, so obstacles are not left floating on slopes.

### Ray casting (commits 15 and 16)

```
struct RayHit { double distance; Vec3 point; Vec3 normal; HitKind kind; int obstacle_index; };
class IScene { virtual bool raycast(origin, direction, max_range, RayHit&) const = 0; ... };
```

- `Scene` builds, at construction, a list of obstacle indices per terrain cell (every cell the obstacle's footprint overlaps).
- A ray is clipped to the scene's xy bounds, then walked cell by cell with a 2D grid traversal. In each cell it tests that cell's two terrain triangles (Moller-Trumbore, inclusive edges so rays through vertices and cell borders do not slip through cracks) and the cell's obstacles. Traversal stops when the best hit so far lies within the cell just processed, which makes the first hit found the nearest one.
- Boxes use the slab method; cylinders use a side quadratic plus two cap discs.
- Terrain is two-sided: a ray from below the surface hits it from below, and normals always face the ray origin. A ray starting inside an obstacle ignores that obstacle (documented; collision avoidance keeps drones out).
- The direction must be a unit vector; a zero vector returns no hit.

### LiDAR (commits 17 and 18)

- `LidarConfig`: beams, azimuth samples, vertical field of view, horizontal field of view, minimum and maximum range, range noise (constant plus per-meter term), base dropout probability, terrain and obstacle reflectivity, detection intensity threshold.
- Scan pattern: beam `b` has elevation evenly spaced across the vertical field of view; sample `k` has azimuth `yaw - fov/2 + (k + 0.5) * fov / N`. Direction is `(cos el cos az, cos el sin az, sin el)`. The sensor is level and yawed with the drone.
- Per ray, fixed draws: one normal (range noise) and one uniform (dropout). A return is valid if there is a hit within range, the hit is beyond the minimum range, the intensity is at or above the threshold, and the dropout draw passes.
- `LidarReturn { valid, range, intensity }`. Ground truth is not in the output.
- Weather (commit 18): `Weather { visibility_m, rain_rate_mm_h }` gives an extinction coefficient `alpha = 3.912 / V + k * rate^gamma`. Intensity of a hit at true range `R` on a surface of reflectivity `rho` is `rho * exp(-2 * alpha * R)`. The maximum detectable range is `ln(rho / threshold) / (2 * alpha)`. The `3.912` factor is the standard Koschmieder relation for a 2% contrast threshold.
- The rain power law constants are placeholders pending validation and are labeled as such in MODELS.md. Only the fog relation gets validated (Phase 7, step "fog visibility fit").

### Jamming zones (commit 19)

- `Jammer`: center, radius (infinite means global), mode (`constant` or `pulsed`), active window `[start_tick, end_tick)`, pulse period, duty, and offset, loss probability inside the zone (default 1), and which endpoint matters (`receiver` by default, or `either`).
- Active at tick `t` if inside the window and, for pulsed, `(t - start + offset) mod period < duty * period`.
- The channel needs endpoint positions. `IAgent` gains a `position()` with a default of the origin, `Simulation` collects ground-truth positions at the barrier, and `IChannel` gains `set_positions`. Jamming is evaluated at send time using the receiver's position then; for messages with a latency of a few ticks this is an approximation, stated in MODELS.md.
- Draw order in `send`: rate limit, base loss, jamming (own stream), jitter. A jammer draws only when it covers the link and its loss probability is below 1.
- `ChannelStats` gains `dropped_jam`, which also enters the simulation hash.
- Jammers are written to the run configuration as repeated `jammer=` lines. This is the one key the config parser allows more than once.

## Commit-by-commit specification

Each commit builds warning-free, adds the listed tests, and passes `ctest`, the invariants script, and the format check. `docs/MODELS.md` gains a section in the commit that adds each model.

### 10. `sim: add Vec3 and point-mass drone dynamics`

Files: `swarm/vec3.hpp`, `swarm/dynamics.hpp`, `src/dynamics.cpp`, `tests/unit/test_vec3.cpp`, `tests/unit/test_dynamics.cpp`, `docs/MODELS.md` (conventions, dynamics).

Tests:
- Vec3: dot, cross (anti-commutative, perpendicular), norm, normalized (zero vector stays zero), clamp_norm (below, at, above limit; direction kept), operators.
- Constant acceleration from rest follows the exact semi-implicit Euler closed form `v = a*n*dt`, `p = a*dt^2*n*(n+1)/2`, and stays close to the continuous result.
- Commanded acceleration above the limit is clamped to the limit with direction preserved.
- Speed saturates at `max_speed`, direction preserved, and applied acceleration falls to zero at saturation.
- No command keeps constant velocity.
- Yaw rate clamp and wrap into `(-pi, pi]`.
- Non-positive `dt` rejected.

### 11. `sim: add IMU model`

Tests:
- Zero noise and bias reports exactly `a - g` and the yaw rate; a stationary drone reads `(0, 0, +9.81)`.
- Sample variance of noise matches `density^2 / dt` within 3% over 200,000 samples; mean near zero.
- Bias random walk: across 2,000 independent IMUs, variance after `n` steps matches `n * sigma_step^2` within 10%.
- Initial bias sigma matches across IMUs.
- Two drone ids give uncorrelated sequences; creating another IMU does not change an existing one's output.
- Same seed and id reproduce exactly; parameters at zero still consume the same draws (stream position equal to a noisy IMU's).
- Negative parameters and non-positive `dt` rejected.
- `Simulation` rejects more than 4096 agents.

### 12. `format: add scene data, serialization, and hash`

Tests:
- Round trip is bit-exact for heights and obstacle fields.
- Hand-written byte layout of a one-cell scene with one box and one cylinder.
- Every proper prefix is reported as truncated; bad magic, unsupported version, inconsistent sizes, and over-limit counts each give their own status.
- `scene_hash` is stable for equal data and changes when any single height or obstacle field changes.
- `validate` rejects NaN, infinity, non-positive cell size, a heights array of the wrong length, and negative sizes.

### 13. `sim: add seeded terrain generator and exact height queries`

Tests:
- Field terrain is exactly zero everywhere.
- Gradient noise is zero at every lattice point and varies between them.
- Hills: same seed gives an identical scene hash, a different seed a different one; `|height| <= amplitude`; mean near zero and spread non-trivial.
- Larger feature scale gives a smoother surface; more octaves adds high-frequency variation (measured by summed neighbor differences).
- Grid dimensions follow the extent rounding rule.
- `height_at` returns the exact plane on a tilted planar test grid, matches grid heights at samples, and is continuous across cell and diagonal boundaries.
- Golden hash of a small fixed hills scene.
- Invalid configurations rejected (non-positive sizes, cell larger than the extent, zero octaves, grid over the limit).

### 14. `sim: add seeded obstacle placement`

Tests:
- Density 0 places nothing; an unsaturated scene places exactly the target count.
- A saturated scene terminates with fewer obstacles and still satisfies spacing.
- All obstacles lie inside the bounds, pairwise spacing is respected, and the spawn area is clear.
- Same seed identical, different seed different.
- Cylinder fraction and size ranges are respected.
- `base_z` equals the embed offset on flat ground and does not exceed the lowest footprint height on slopes.
- Resulting `SceneData` validates.
- Invalid ranges and fractions rejected.

### 15. `sim: add terrain ray casting`

Tests:
- Vertical and oblique hits on flat ground at the analytic distance, with an upward-facing normal.
- Ray parallel to the ground, pointing away, or beyond `max_range` misses.
- Tilted planar terrain gives the analytic distance; rays crossing many cells diagonally, along cell borders, and through vertices still hit.
- Ray starting outside the bounds and entering; ray leaving the bounds; ray starting below the surface (hits from below, normal faces the origin).
- Zero direction returns no hit.
- Agreement with a brute-force test over every triangle on 20,000 random rays on a generated hills scene, to 1e-9.

### 16. `sim: add obstacle ray casting and unified traversal`

Tests:
- Each of a box's six faces, hit from outside along its axis, with correct distance and normal; a cylinder's side and both caps likewise.
- Nearest of terrain and obstacle wins; an obstacle hidden behind a hill is not hit; an obstacle spanning several cells is found from every approach.
- A ray starting inside an obstacle ignores it.
- Agreement with brute force (every triangle and every obstacle) on 50,000 random rays against a scene with 300 obstacles, including hit kind and obstacle index.
- Results identical for different internal cell sizes of the same geometry.
- Query is `const` and works from multiple threads (full concurrency test in commit 20).
- A small benchmark program (`swarm_bench`) is added to measure rays per second; its numbers go into the docs only when measured.

### 17. `sim: add LiDAR scan pattern, range noise, and dropouts`

Tests:
- Beam elevations and azimuths follow the pattern; directions are unit length; one beam uses the mid elevation.
- Downward beams over flat ground read `h / sin(elevation)` with zero noise; upward beams are invalid; counts equal beams times samples.
- Yaw by one azimuth step shifts the pattern by one sample on a rotationally symmetric scene.
- Range noise standard deviation matches the configured constant and per-meter terms; dropout rate matches; minimum and maximum range respected.
- A different scene leaves the noise on unrelated rays unchanged (fixed draws).
- Two drones' LiDARs are independent; same seed reproduces exactly.
- Invalid configurations rejected.

### 18. `sim: add weather extinction and detection threshold to LiDAR`

Tests:
- `alpha = 3.912 / V` for fog; rain adds `k * rate^gamma`; zero rain constants have no effect; fog and rain add.
- Maximum detectable range matches `ln(rho / threshold) / (2 * alpha)`.
- A wall just inside that range is detected and just outside is not.
- Clear weather leaves a standard scan unchanged from commit 17.
- Intensity equals `rho * exp(-2 * alpha * R)`.
- Weather changes which returns are valid but not the noise drawn for rays that remain valid in both cases.
- Invalid weather (non-positive visibility, negative rain) rejected.

### 19. `channel: add jamming zones`

Tests:
- Receiver inside the radius is dropped; outside is delivered; exactly on the boundary counts as inside.
- Sender inside is unaffected with `receiver` mode and affected with `either`.
- Nothing is dropped before the window starts or after it ends.
- A pulsed jammer is active for exactly `duty * period` ticks per period, and the offset shifts the pattern.
- Loss probability 0.5 inside the zone gives about 50% drops; probability 1 consumes no random draws.
- Adding a jammer does not change which messages outside the zone are lost.
- Several jammers combine (any one can drop a message).
- Moving a receiver in and out of the zone changes its fate between ticks.
- Missing positions with jammers configured is an error.
- Jammer validation (negative radius, duty outside [0, 1], zero period, empty window, loss outside [0, 1], NaN).
- `jammer=` lines round-trip through the run configuration; malformed lines are rejected.
- In the simulation: agents inside a global or local zone receive nothing during the window, and hashes are identical for 1, 2, and 8 threads.

### 20. `test: run sensors and dynamics on the lockstep loop`

Test-only commit plus a test support agent that combines dynamics, IMU, and LiDAR on a shared scene.

Tests:
- Final and per-tick hashes identical for 1, 2, 4, and 8 threads with a generated hills-and-obstacles scene, fog, jammers, and burst loss, 16 drones over 200 ticks.
- Same seed reproduces; a different scene seed changes the hash.
- A shared `Scene` queried by 100,000 rays from pool threads returns the same results as the sequential run.
- The ThreadSanitizer job runs all of this.

## Documentation produced

- `docs/MODELS.md`: for each model, the equations, parameters with units, assumptions, known limitations, and validation status. The weather section states plainly which constants are unvalidated placeholders.

## Risks

| Risk | Handling |
|---|---|
| Ray traversal edge cases (vertices, borders, grazing) | Inclusive triangle tests and large brute-force comparisons on random rays |
| Terrain and Unreal mesh drift | One shared triangulation rule, `SceneData` readable from `core/format`, golden hash |
| Statistical tests flaking | Fixed seeds, tolerances sized from sample counts and written in the tests |
| Hidden shared state in the scene | `const` interface without mutable members, concurrency test, TSan job |
| Placeholder weather constants mistaken for results | Labeled in MODELS.md and the README limitations; fog validated in Phase 7 |
| Scope growth into estimation | Sensor output is not consumed anywhere except tests until Phase 3 |

## Exit criteria

- All eleven commits made, each green locally and in CI (gcc, clang, sanitizers, C++20).
- `docs/MODELS.md` covers every model added.
- Sensors on the lockstep loop produce identical hashes for any thread count.
- PLAN.md change log and README status updated; benchmark numbers recorded only if measured.
