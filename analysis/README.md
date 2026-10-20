# Analysis

Reads the results table written by `swarm_batch` and computes summaries. It does not run the simulation. The setup and commands are in [docs/DEVELOPMENT.md](../docs/DEVELOPMENT.md), and the definitions are in [docs/MODELS.md](../docs/MODELS.md).

| Module | What it does |
|---|---|
| `table.py` | Reads the CSV. The axes are the columns between `cell` and `replicate`; an empty metric cell is NaN, never zero |
| `stats.py` | Mean, standard deviation, a 95 percent Student t interval for the mean, median, and range, with the counts of values used and missing |
| `aggregate.py` | Groups runs by axis levels, summarizes metrics, and computes paired differences between two levels of an axis |

`python -m swarmsim_analysis summarize TABLE --by AXES --metrics NAMES --out FILE` writes one row per group.
