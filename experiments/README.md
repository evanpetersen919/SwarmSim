# Experiments

Experiment files for the batch tools. The format is described in [docs/MODELS.md](../docs/MODELS.md) and the design in [docs/phases/phase-07-batch-and-analysis.md](../docs/phases/phase-07-batch-and-analysis.md).

| File | What it is |
|---|---|
| `base.yaml` | The scenario that the experiments vary |
| `matrix.yaml` | The planned matrix: loss, weather, jamming, drone count, faults, and scene; 432 cells |
| `smoke.yaml` | Eight short runs, for checking the tools |
| `fog_convention.yaml` | The fog constant 3.912 against 3.0 at six visibilities; where the position fix is lost |
| `ablation.yaml` | The full controller and three variants that each disable one component; 48 cells |
| `baselines.yaml` | Hold-last, dead reckoning, Kalman, and the oracle predictors; 48 cells |
| `ablation_fog.yaml` | The fallback state machine in fog thick enough to lose the LiDAR position fix: the full controller and the variant without the fallback, four visibilities, with and without pulsed jamming; 16 cells |
| `baselines_burst.yaml` | The same four predictors under bursty loss (mean burst of 6) at the same average rates; 48 cells |
| `baselines_drop.yaml` | `baselines.yaml` with the stale-neighbor rule off and on (a neighbor silent for more than 100 ticks leaves the formation consensus); 96 cells |
| `baselines_burst_drop.yaml` | The same with bursty loss; 96 cells |
| `probe_fixed_wing_map_errors.yaml` | A probe of how wrong the prior map must be before aircraft without a forward sensor reach the ground; 13 cells, five seeds, not a result (ADR 0013) |
| `fixed_wing_forward.yaml` | The forward obstacle sensor experiment: 8 map levels, 4 sensor arms, 2 weather levels; 64 cells (ADR 0013) |
| `fixed_wing_forward_sensitivity.yaml` | The forward sensor's range and dropout against four maps; 36 cells (ADR 0013) |
| `base_fixed_wing_avoid.yaml` | Six ALTIUS-700-class aircraft at 70 mph on a 300 m orbit over generated hills, flown at 100 m with no map (ADR 0014) |
| `probe_fixed_wing_avoid.yaml` | A probe of how steep the hills must be before aircraft without avoidance reach the ground; 12 cells, five seeds, not a result (ADR 0014) |
| `fixed_wing_avoid.yaml` | The avoidance experiment: 4 steepness levels, 4 maps, 3 sensor arms, 2 weather levels; 96 cells (ADR 0014) |
| `fixed_wing_avoid_sensitivity.yaml` | The avoidance rule's horizon and bank fraction on 400 m hills; 12 cells (ADR 0014) |
| `fixed_wing_avoid_rejoin.yaml` | The rejoin experiment: 4 steepness levels, 2 maps, 2 weather levels, 7 arms; 112 cells (ADR 0015) |
| `fixed_wing_avoid_urgent.yaml` | The urgent-turn experiment: 2 weather levels, 3 steepness levels, 2 maps, 4 arms (urgent_s of 0, 4, 6, 8 s); 48 cells (ADR 0017) |
| `fixed_wing_avoid_turnfirst.yaml` | The turn-first experiment: 2 weather levels, 3 steepness levels, 2 maps, 3 arms (all3, turnfirst, turnfirst_urgent6); 36 cells (ADR 0018) |
| `probe_fixed_wing_phase.yaml` | A probe of the orbit phase gain (0.05, 0.02, 0) on 400 and 480 m hills in fog; 6 cells (ADR 0019) |
| `fixed_wing_avoid_catchup.yaml` | The catch-up experiment: 2 weather levels, 3 steepness levels, 2 maps, 4 arms (no cap, 6, 3, and 0 m/s); 48 cells (ADR 0019) |
| `fixed_wing_avoid_clearance.yaml` | The avoidance clearance (15, 30, 60 m) with the three changes of ADR 0015; 6 cells |
| `fixed_wing_share.yaml` | The sharing experiment: 3 steepness levels, 2 weather levels, 3 loss levels, 2 jam levels, 3 arms (avoid, memory, share); 108 cells (ADR 0016) |
| `fixed_wing_share_sensitivity.yaml` | How long a point is kept and how many a report carries; 6 cells (ADR 0016) |
| `fixed_wing_share_trust.yaml` | One aircraft sending false hazards, with and without a confirmation rule of two; 32 cells (ADR 0016) |
| `base_fixed_wing.yaml` | The fixed-wing scenario (scenario version 2) that the fixed-wing experiments vary |
| `matrix_fixed_wing.yaml` | Weather, jamming, loss, drone count, and faults for the default fixed-wing controller; 216 cells |
| `baselines_fixed_wing.yaml` | Four neighbor predictors against three consensus weights, under jamming and loss; 72 cells |
| `ablation_fixed_wing.yaml` | The full controller and three variants, with and without altitude layers, in three weathers, with and without a failure; 48 cells |
| `base_fixed_wing_energy.yaml` | The fixed-wing scenario on the current model (scenario version 3: energy dynamics, wind, ground contact, terrain guidance, 120 m hills) |
| `pilot_fixed_wing.yaml` | A pilot on the current model: weather, radar altimeter, and loss; 8 cells, 40 runs. See docs/phases/flight-realism.md |
| `base_fixed_wing_dense.yaml` | Twelve fixed-wing aircraft on one 80 m orbit, about 41 m apart so that separation acts (scenario version 3) |
| `base_fixed_wing_forward.yaml` | The dense base with the mission altitude at 20 m, below the clearance, so the prior map decides the altitude flown (ADR 0013) |
| `pilot_fixed_wing_comms.yaml` | A comms pilot on that scenario: weather, neighbor predictor, and loss; 24 cells, 120 runs |
| `pilot_fixed_wing_altimeter.yaml` | An altimeter pilot on the dense scenario: weather and radar altimeter; 4 cells, 40 runs |
| `probe_fixed_wing.yaml` | A probe of 384 cells that shaped the fixed-wing matrix; see docs/phases/fixed-wing-vehicle.md |

The fixed-wing experiments use scenario version 2, the coordinated-turn model without wind, and were run before the energy model, wind, and terrain guidance existed (ADR 0011). They are not results for the current model; the reruns are listed in [docs/PHASES.md](../docs/PHASES.md).

A `base` path is relative to the experiment file.
