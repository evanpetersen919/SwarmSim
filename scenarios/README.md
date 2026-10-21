# Scenarios

Scenario files for `swarm_sim`. The format is described in [docs/MODELS.md](../docs/MODELS.md) and the decision record [0007](../docs/decisions/0007-strict-versioned-scenario-files.md). Unknown keys are errors, so a misspelled key is reported, not ignored.

| File | What it exercises |
|---|---|
| `minimal.yaml` | Only the required keys; every other setting takes its default |
| `formation_jam.yaml` | Eight drones assembling into a wedge over open field, burst packet loss, and a six second jam of every link |
| `hills_fog_fault.yaml` | Six drones in a grid over hills in fog and rain, independent loss, a pulsed local jammer, and one drone failing |
| `hold_resume.yaml` | A ground station holds six drones at 30 s and resumes them at 36 s, through half the beacons lost in bursts and a jam of every link |
| `fixed_wing_orbit.yaml` | Eight fixed-wing aircraft of the ALTIUS class orbiting a point over hills in haze, with camera position fixes, a constant jammer, and independent packet loss (scenario version 2) |
