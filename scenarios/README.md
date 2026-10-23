# Scenarios

Scenario files for `swarm_sim`. The format is described in [docs/MODELS.md](../docs/MODELS.md) and the decision record [0007](../docs/decisions/0007-strict-versioned-scenario-files.md). Unknown keys are errors, so a misspelled key is reported, not ignored.

| File | What it exercises |
|---|---|
| `minimal.yaml` | Only the required keys; every other setting takes its default |
| `formation_jam.yaml` | Eight drones assembling into a wedge over open field, burst packet loss, and a six second jam of every link |
| `hills_fog_fault.yaml` | Six drones in a grid over hills in fog and rain, independent loss, a pulsed local jammer, and one drone failing |
| `hold_resume.yaml` | A ground station holds six drones at 30 s and resumes them at 36 s, through half the beacons lost in bursts and a jam of every link |
| `fixed_wing_orbit.yaml` | Eight fixed-wing aircraft of the ALTIUS class orbiting a point over hills in haze, with camera position fixes, a constant jammer, and independent packet loss (scenario version 2) |
| `fixed_wing_flight.yaml` | Eight fixed-wing aircraft on a racetrack launched 12 s apart over hills, with the energy model, a steady wind with gusts, a terrain map that raises their altitude over high ground, and ground contact that ends a flight (scenario version 3) |
| `fixed_wing_rake.yaml` | Fifteen fixed-wing aircraft of ALTIUS-700-class (figures supplied without a source, unverified) flying a broadside rake with a 739 m spacing from the sensor swath and a 20 m altitude stagger, for 90 s over generated hills (scenario version 3) |
| `fixed_wing_formation.yaml` | Four fixed-wing aircraft flying the formation sequence reported for four ALTIUS-600 at EDGE 22 (an echelon, a left echelon, a vee) at the published 60 km/h cruise speed, for 60 s (scenario version 3) |
