"""Compares the top surface of an Unreal level with the scene the simulation runs on.

Usage:
    python scripts/audit_export.py SCENE.swscene AUDIT.csv [--center X,Y --radius R --band W]
                                                           [--threshold M]

SCENE.swscene is the scene file a scenario uses. AUDIT.csv is written by the viewer with
`-SwarmAuditScene=PATH` (and the same -SwarmAnchor, -SwarmExportSize, and -SwarmExportCell the scene
was exported with): the highest thing a trace straight down meets at each point of the grid, which
is what an aircraft over that point would hit. For every point the script compares it with

  scene: the simulation's surface, the terrain and the top of any obstacle (cylinder or box) that
         contains the point (`Scene::surface_height`), which is what the simulated ground contact
         and sensors see; and
  map:   the prior map an aircraft plans with, the terrain with each obstacle's top raised into
         every sample under its bounding box, the highest corner of the cell the point is in
         (`make_terrain_map`, `TerrainMap::max_height_near` at a point).

It prints how far the level rises above each, in meters, over the whole area and in a corridor
around a circle if one is given (an orbit), and which meshes are the highest thing at the points
where the level rises most above the scene. Standard library only.
"""

import argparse
import csv
import math
import struct
import sys
from collections import Counter, defaultdict


def read_scene(path):
    data = open(path, "rb").read()
    pos = 4  # magic
    version, cells_x, cells_y = struct.unpack_from("<III", data, pos)
    pos += 12
    origin_x, origin_y, cell = struct.unpack_from("<ddd", data, pos)
    pos += 24
    (count,) = struct.unpack_from("<I", data, pos)
    pos += 4
    samples = (cells_x + 1) * (cells_y + 1)
    heights = struct.unpack_from("<%dd" % samples, data, pos)
    pos += 8 * samples
    obstacles = []
    for _ in range(count):
        (kind,) = struct.unpack_from("<B", data, pos)
        pos += 1
        cx, cy, base, height, half_x, half_y = struct.unpack_from("<6d", data, pos)
        pos += 48
        obstacles.append((kind, cx, cy, base, height, half_x, half_y))
    return {
        "cells_x": cells_x,
        "cells_y": cells_y,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "cell": cell,
        "heights": list(heights),
        "obstacles": obstacles,
    }


def terrain_height(scene, x, y):
    stride = scene["cells_x"] + 1
    fx = (x - scene["origin_x"]) / scene["cell"]
    fy = (y - scene["origin_y"]) / scene["cell"]
    fx = min(max(fx, 0.0), scene["cells_x"] - 1e-9)
    fy = min(max(fy, 0.0), scene["cells_y"] - 1e-9)
    i, j = int(fx), int(fy)
    u, v = fx - i, fy - j
    h = scene["heights"]
    ha = h[j * stride + i]
    hb = h[j * stride + i + 1]
    hc = h[(j + 1) * stride + i + 1]
    hd = h[(j + 1) * stride + i]
    if u >= v:  # triangle (a, b, c)
        return ha + u * (hb - ha) + v * (hc - hb)
    return ha + u * (hc - hd) + v * (hd - ha)  # triangle (a, c, d)


def raised_map(scene):
    """The map's samples: terrain with each obstacle's top raised under its bounding box."""
    stride = scene["cells_x"] + 1
    rows = scene["cells_y"] + 1
    heights = list(scene["heights"])
    cell = scene["cell"]
    for kind, cx, cy, base, height, half_x, half_y in scene["obstacles"]:
        top = base + height
        hy = half_x if kind == 1 else half_y
        i0 = max(0, math.floor((cx - half_x - scene["origin_x"]) / cell))
        i1 = min(stride - 1, math.ceil((cx + half_x - scene["origin_x"]) / cell))
        j0 = max(0, math.floor((cy - hy - scene["origin_y"]) / cell))
        j1 = min(rows - 1, math.ceil((cy + hy - scene["origin_y"]) / cell))
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                k = j * stride + i
                if heights[k] < top:
                    heights[k] = top
    return heights


def map_height(scene, raised, x, y):
    stride = scene["cells_x"] + 1
    fx = (x - scene["origin_x"]) / scene["cell"]
    fy = (y - scene["origin_y"]) / scene["cell"]
    fx = min(max(fx, 0.0), scene["cells_x"] - 1e-9)
    fy = min(max(fy, 0.0), scene["cells_y"] - 1e-9)
    i, j = int(fx), int(fy)
    return max(raised[j * stride + i], raised[j * stride + i + 1], raised[(j + 1) * stride + i],
               raised[(j + 1) * stride + i + 1])


def obstacle_index(scene):
    """Obstacles by the grid cell of their center, for a point lookup."""
    grid = defaultdict(list)
    cell = scene["cell"]
    for ob in scene["obstacles"]:
        grid[(math.floor((ob[1] - scene["origin_x"]) / cell),
              math.floor((ob[2] - scene["origin_y"]) / cell))].append(ob)
    return grid


def scene_height(scene, grid, x, y):
    best = terrain_height(scene, x, y)
    cell = scene["cell"]
    ci = math.floor((x - scene["origin_x"]) / cell)
    cj = math.floor((y - scene["origin_y"]) / cell)
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            for kind, cx, cy, base, height, half_x, half_y in grid.get((ci + di, cj + dj), ()):
                if kind == 1:
                    inside = (x - cx) ** 2 + (y - cy) ** 2 <= half_x ** 2
                else:
                    inside = abs(x - cx) <= half_x and abs(y - cy) <= half_y
                if inside:
                    best = max(best, base + height)
    return best


def percentile(sorted_values, q):
    if not sorted_values:
        return float("nan")
    return sorted_values[min(len(sorted_values) - 1, int(q * len(sorted_values)))]


def report(title, rows, threshold):
    if not rows:
        print(f"{title}: no points")
        return
    for label, index in (("scene", 3), ("map", 4)):
        values = sorted(r[index] for r in rows)
        over = sum(1 for v in values if v > threshold)
        print(f"{title}, level above {label}: points {len(values)}, above {threshold:g} m: "
              f"{over} ({100.0 * over / len(values):.2f} percent), median {percentile(values, 0.5):.2f} m, "
              f"99th percentile {percentile(values, 0.99):.2f} m, max {values[-1]:.2f} m")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("scene")
    parser.add_argument("audit")
    parser.add_argument("--center", default=None, help="x,y of a circle in the simulation frame")
    parser.add_argument("--radius", type=float, default=300.0)
    parser.add_argument("--band", type=float, default=60.0, help="half width of the corridor, m")
    parser.add_argument("--threshold", type=float, default=5.0)
    args = parser.parse_args()

    scene = read_scene(args.scene)
    raised = raised_map(scene)
    grid = obstacle_index(scene)
    print(f"scene: {scene['cells_x']} x {scene['cells_y']} cells of {scene['cell']:g} m, "
          f"{len(scene['obstacles'])} obstacles, tallest "
          f"{max((o[4] for o in scene['obstacles']), default=0.0):.1f} m")

    rows = []  # x, y, name, above scene, above map
    with open(args.audit, newline="") as handle:
        for r in csv.DictReader(handle):
            x, y, top = float(r["x_m"]), float(r["y_m"]), float(r["top_m"])
            rows.append((x, y, r["name"], top - scene_height(scene, grid, x, y),
                         top - map_height(scene, raised, x, y)))
    report("whole area", rows, args.threshold)
    if args.center:
        cx, cy = (float(v) for v in args.center.split(","))
        corridor = [r for r in rows
                    if abs(math.hypot(r[0] - cx, r[1] - cy) - args.radius) <= args.band]
        report(f"corridor (radius {args.radius:g} m, +/- {args.band:g} m)", corridor, args.threshold)
    else:
        corridor = rows

    print("\nthe highest thing at the points where the level is more than "
          f"{args.threshold:g} m above the scene ({'corridor' if args.center else 'whole area'}):")
    counts = Counter()
    worst = {}
    for x, y, name, above_scene, above_map in corridor:
        if above_scene > args.threshold:
            counts[name] += 1
            if name not in worst or above_scene > worst[name][0]:
                worst[name] = (above_scene, x, y)
    for name, n in counts.most_common(15):
        above, x, y = worst[name]
        print(f"  {name}: {n} points, up to {above:.1f} m above the scene at ({x:.0f}, {y:.0f})")
    if not counts:
        print("  none")


if __name__ == "__main__":
    sys.exit(main())
