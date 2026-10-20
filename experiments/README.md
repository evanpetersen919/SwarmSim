# Experiments

Experiment files for the batch tools. The format is described in [docs/MODELS.md](../docs/MODELS.md) and the design in [docs/phases/phase-07-batch-and-analysis.md](../docs/phases/phase-07-batch-and-analysis.md).

| File | What it is |
|---|---|
| `base.yaml` | The scenario that the experiments vary |
| `matrix.yaml` | The planned matrix: loss, weather, jamming, drone count, faults, and scene; 432 cells |
| `smoke.yaml` | Eight short runs, for checking the tools |
| `ablation.yaml` | The full controller and three variants that each disable one component; 48 cells |
| `baselines.yaml` | Hold-last, dead reckoning, Kalman, and the oracle predictors; 48 cells |

A `base` path is relative to the experiment file.
