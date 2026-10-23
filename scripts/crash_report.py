"""Says why each aircraft that reached the ground did, from the flown paths and the scene.

Usage:
    python scripts/crash_report.py SWARM_SIM SCENARIO.yaml --seeds 30 [--work DIR] [--every N]

Runs the scenario for seeds 1 to N (the scenario text with its `seed:` replaced), reads each run's
positions (`swarm_sim positions`) and scene (`swarm_sim export-scene`), and for every aircraft that
stops on the ground prints what it was doing in the last seconds and puts it in one class:

  launch       it was on the ground in the first record
  early        it stopped within EARLY_S of the start (the ground was close from the start, and the
               sensor had had no time)
  separation   another aircraft was within SEPARATION_M in the last 3 s
  climb        it was climbing at (nearly) its limit and the ground ahead rose faster
  turn         it was turning at (nearly) its sideways limit and the ground was still ahead
  blind        the point where it hit was outside the sensor's field of view 3 s before
  other        none of those

The classes are tests on the flown path, not on what the brain sensed, which the log does not hold,
so a crash with no class is one whose cause the path does not show. Standard library only.
"""

import argparse
import csv
import math
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import audit_export  # noqa: E402

SEPARATION_M = 60.0
EARLY_S = 5.0
FAN_HALF_DEG = 20.0
CLIMB_LIMIT_MPS = 3.0
CLIMB_NEAR = 0.8  # of the limit
LATERAL_LIMIT = 6.85  # g * tan(0.61), m/s^2
LATERAL_NEAR = 0.7


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout


def load_positions(path):
    rows = {}
    with open(path, newline="") as handle:
        for r in csv.DictReader(handle):
            rows.setdefault(int(r["drone"]), []).append(
                (float(r["time_s"]), float(r["x_m"]), float(r["y_m"]), float(r["z_m"]), r["alive"]))
    return rows


def crash_index(track):
    """The index at which the aircraft stops for good, or None."""
    last = len(track) - 1
    if last < 5:
        return None
    end = track[last]
    k = last
    while k > 0 and math.dist(track[k][1:4], end[1:4]) < 1e-6:
        k -= 1
    # The log's last record repeats the one before it, so a moving aircraft is stationary for two
    # records. It has crashed if it stayed put for at least five.
    if last - k < 5:
        return None
    return k + 1


def classify(drone, tracks, k, scene, dt_s):
    track = tracks[drone]
    t, x, y, z, _ = track[k]
    info = {"drone": drone, "time_s": t}
    if t <= 0.5:
        info["class"] = "launch"  # it stopped at the first step: it started on the ground
        return info
    window = max(0, k - int(round(10.0 / dt_s)))  # the last 10 s
    # Speeds over the last seconds.
    def velocity(i):
        a, b = track[i], track[i + 1]
        h = b[0] - a[0]
        return ((b[1] - a[1]) / h, (b[2] - a[2]) / h, (b[3] - a[3]) / h)
    vs = [velocity(i) for i in range(window, k - 1)] or [(0.0, 0.0, 0.0)]
    near = vs[-max(1, int(round(3.0 / dt_s))):]
    climb = sum(v[2] for v in near) / len(near)
    speed = sum(math.hypot(v[0], v[1]) for v in near) / len(near)
    # Sideways acceleration over the last 3 s: the change of the velocity vector's direction.
    turn_rates = []
    for a, b in zip(near[:-1], near[1:]):
        h = math.atan2(b[1], b[0]) - math.atan2(a[1], a[0])
        h = (h + math.pi) % (2 * math.pi) - math.pi
        turn_rates.append(abs(h) / dt_s)
    lateral = speed * (sum(turn_rates) / len(turn_rates)) if turn_rates else 0.0
    info.update(speed=round(speed, 1), climb=round(climb, 2), lateral=round(lateral, 2))
    # The ground ahead along the heading, 100 m on, against where the aircraft is.
    heading = math.atan2(near[-1][1], near[-1][0])
    ahead = audit_export.terrain_height(scene, x + 100.0 * math.cos(heading), y + 100.0 * math.sin(heading))
    here = audit_export.terrain_height(scene, x, y)
    slope = (ahead - here) / 100.0
    info["slope"] = round(slope, 2)
    info["height_before_m"] = round(track[max(0, k - int(round(5.0 / dt_s)))][3]
                                    - audit_export.terrain_height(scene, *track[max(0, k - int(round(5.0 / dt_s)))][1:3]), 1)
    # Where the impact point was, 3 s and 6 s before: its bearing from the heading then, its
    # distance, and the climb that clearing it would have needed against what 3 m/s allows.
    for label, back in (("3s", 3.0), ("6s", 6.0)):
        i = max(0, k - int(round(back / dt_s)))
        j = min(i + 1, k)
        h = (track[j][0] - track[i][0]) or dt_s
        vx, vy = (track[j][1] - track[i][1]) / h, (track[j][2] - track[i][2]) / h
        dx, dy = x - track[i][1], y - track[i][2]
        dist = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(vx * dy - vy * dx, vx * dx + vy * dy)) if dist > 1e-6 else 0.0
        sp = math.hypot(vx, vy) or 1.0
        needed = z + 30.0 - track[i][3]
        reachable = CLIMB_LIMIT_MPS * (dist / sp)
        info["dist_" + label] = round(dist)
        info["bearing_" + label] = round(bearing)
        info["need_over_reach_" + label] = round(needed / reachable, 1) if reachable > 0 else None
    # Another aircraft close in the last 3 s?
    close = None
    for other, other_track in tracks.items():
        if other == drone:
            continue
        for i in range(max(0, k - int(round(3.0 / dt_s))), min(k + 1, len(other_track))):
            d = math.dist(track[i][1:4], other_track[i][1:4])
            if close is None or d < close:
                close = d
    info["nearest_aircraft_m"] = None if close is None else round(close, 1)
    if t <= EARLY_S:
        info["class"] = "early"  # within seconds of the start: the ground was close from the start
    elif close is not None and close < SEPARATION_M:
        info["class"] = "separation"
    elif climb >= CLIMB_NEAR * CLIMB_LIMIT_MPS and slope * speed > climb:
        info["class"] = "climb"
    elif lateral >= LATERAL_NEAR * LATERAL_LIMIT:
        info["class"] = "turn"
    elif abs(info["bearing_3s"]) > FAN_HALF_DEG:
        info["class"] = "blind"  # the impact point was outside the sensor's field 3 s before
    else:
        info["class"] = "other"
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("sim")
    parser.add_argument("scenario")
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--work", default="crash_report_work")
    parser.add_argument("--every", type=int, default=5, help="ticks between positions read")
    args = parser.parse_args()
    os.makedirs(args.work, exist_ok=True)
    text = open(args.scenario, encoding="utf-8").read()
    events = []
    runs = 0
    aircraft = 0
    for seed in range(1, args.seeds + 1):
        yaml_path = os.path.join(args.work, f"run_{seed}.yaml")
        log_path = os.path.join(args.work, f"run_{seed}.swlog")
        pos_path = os.path.join(args.work, f"run_{seed}.csv")
        scene_path = os.path.join(args.work, f"run_{seed}.swscene")
        with open(yaml_path, "w", encoding="utf-8") as handle:
            handle.write(re.sub(r"(?m)^seed:.*$", f"seed: {seed}", text, count=1))
        run([args.sim, "run", yaml_path, "--out", log_path])
        run([args.sim, "positions", log_path, "--csv", pos_path, "--every", str(args.every)])
        run([args.sim, "export-scene", log_path, "--out", scene_path])
        scene = audit_export.read_scene(scene_path)
        tracks = load_positions(pos_path)
        dt_s = tracks[0][1][0] - tracks[0][0][0]
        runs += 1
        aircraft += len(tracks)
        for drone, track in tracks.items():
            k = crash_index(track)
            if k is None:
                continue
            info = classify(drone, tracks, k, scene, dt_s)
            info["seed"] = seed
            events.append(info)
    print(f"{runs} runs, {aircraft} aircraft, {len(events)} stopped on the ground")
    counts = {}
    for e in events:
        counts[e["class"]] = counts.get(e["class"], 0) + 1
    for name in ("launch", "early", "separation", "climb", "turn", "blind", "other"):
        print(f"  {name:10s} {counts.get(name, 0)}")
    print("\nseed drone  time_s  class       speed  climb  lateral  slope  dist3 bear3 need/reach3 "
          "dist6 bear6 need/reach6  nearest_aircraft")
    for e in sorted(events, key=lambda e: (e["class"], e["seed"], e["drone"])):
        if e["class"] == "launch":
            continue
        print(f"{e['seed']:4d} {e['drone']:5d} {e['time_s']:7.1f}  {e['class']:10s} "
              f"{e.get('speed', '')!s:>6} {e.get('climb', '')!s:>6} {e.get('lateral', '')!s:>8} "
              f"{e.get('slope', '')!s:>6} {e.get('dist_3s', '')!s:>6} {e.get('bearing_3s', '')!s:>5} "
              f"{e.get('need_over_reach_3s', '')!s:>11} {e.get('dist_6s', '')!s:>5} "
              f"{e.get('bearing_6s', '')!s:>5} {e.get('need_over_reach_6s', '')!s:>11} "
              f"{e.get('nearest_aircraft_m', '')!s:>16}")


if __name__ == "__main__":
    main()
