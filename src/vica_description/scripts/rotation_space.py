#!/usr/bin/env python3
"""How much of a map can this robot turn around in?

    ros2 run vica_description rotation_space <map.yaml> [more maps...]

Reads only. No robot, no simulator, no Nav2 -- it is geometry against an image,
so it gives the same answer every time, which none of the driving measurements
here do.

The question matters because a great deal of effort can go into making a
controller turn in place before anyone checks whether there is room to. The
robot's own notes put "제자리 회전 가능 면적" at 42.0 to 44.0 % of free area,
which sounds like a limitation to work around rather than one to design for.

That figure is measured from the wrong centre, though, and this recomputes it
from the right one.

A differential drive commanded to spin does not turn about the middle of its
body. It turns about the midpoint of the drive axle, which on VICA sits 0.154 m
forward of base_footprint. The tail is 0.595 m behind base_footprint, so from
the actual pivot it is 0.749 m out, and it is the tail that sweeps:

    about base_footprint     circumscribed 0.646 m -> needs 1.29 m of width
    about the drive axle     circumscribed 0.800 m -> needs 1.60 m

Both are printed, because the difference between them is the point. The robot's
own record of median corridor half-width is 0.70 m, so a typical corridor is
1.40 m across -- wide enough on the first number, not on the second.

Measured on vica_map_0630, whose free area of 106.4 m2 matches the figure in the
robot's own notes exactly:

                          body centre    drive axle
    hexagon (dev)         40.6 %         31.8 %
    rectangle (deployed)  38.7 %         30.9 %

The left column reproduces the 42-44 % on record, which is how we know that is
the centre it was computed from. The right column is the one that governs. Just
under a third of the space the robot drives in is space it can turn around in.

The simulated hospital gives 53.2 % on the same measure against 273.4 m2 free,
so it is a markedly more forgiving building than the real site. A rotation
result obtained there does not carry over on its own.

A cell counts as rotatable when every cell within the circumscribed radius is
free. That is the right test for a full turn: the swept region of a body
rotating about a point inside itself is the whole disc, not a ring.

Unknown cells count as blocked. A map's unknown space is usually outside the
walls or behind furniture, and treating it as free would inflate the answer
exactly where the robot has least business turning.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

# Both workspaces' footprints, and the pivot they actually rotate about.
FOOTPRINTS = {
    "hexagon (dev, both workspaces)": [
        (0.305, 0.2275), (0.305, -0.2275), (-0.305, -0.2275),
        (-0.595, -0.035), (-0.595, 0.035), (-0.305, 0.2275),
    ],
    "rectangle (deployed config)": [
        (0.305, 0.227), (0.305, -0.227), (-0.565, -0.227), (-0.565, 0.227),
    ],
}
FOOTPRINT_PADDING = 0.05

# base_link's drive joints sit at x = +0.154, and base_footprint is directly
# below base_link, so the axle midpoint is +0.154 in the footprint frame too.
DRIVE_AXLE_X = 0.154


def circumscribed(points, pivot_x, padding=FOOTPRINT_PADDING):
    return max(math.hypot(x - pivot_x, y) for x, y in points) + padding


def read_map(yaml_path):
    with open(yaml_path) as fh:
        meta = yaml.safe_load(fh)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)

    with open(image, "rb") as fh:
        magic = fh.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"{image}: expected a binary or ASCII PGM, got {magic}")
        dims = []
        while len(dims) < 3:
            line = fh.readline()
            if line.startswith(b"#"):
                continue
            dims += line.split()
        width, height, maxval = (int(v) for v in dims[:3])
        if magic == b"P5":
            data = np.frombuffer(fh.read(width * height), dtype=np.uint8)
        else:
            data = np.array(fh.read().split(), dtype=np.uint8)
        grid = data.reshape(height, width)

    # 205 is the value map_saver writes for never-observed cells, and it has to
    # be handled by its meaning rather than by the thresholds. Run through the
    # usual arithmetic it comes out at (255-205)/255 = 0.196, under a free_thresh
    # of 0.25, so a threshold-only reading calls unobserved space free. On this
    # map that is 82 % of the image, and it turned a 626.8 m2 answer out of a
    # map whose free area the robot's own notes put at 106.4 m2.
    UNKNOWN_VALUE = 205
    occ = (maxval - grid.astype(float)) / maxval
    if meta.get("negate", 0):
        occ = 1.0 - occ
    unknown = grid == UNKNOWN_VALUE
    occupied = (occ > meta.get("occupied_thresh", 0.65)) & ~unknown
    free = ~unknown & ~occupied
    return meta, free, occupied, unknown


def rotatable(free, blocked, radius_m, resolution):
    """Cells whose whole disc of `radius_m` is free.

    Done as a min-filter over the disc: a cell survives when no blocked cell
    lies within the radius. scipy would be one call, but it is not a dependency
    of this package and the maps are small enough to do it by shifting.
    """
    r = int(math.ceil(radius_m / resolution))
    h, w = free.shape
    near_blocked = np.zeros_like(blocked)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if math.hypot(dx, dy) * resolution > radius_m:
                continue
            ys0, ys1 = max(0, dy), min(h, h + dy)
            xs0, xs1 = max(0, dx), min(w, w + dx)
            yd0, yd1 = max(0, -dy), min(h, h - dy)
            xd0, xd1 = max(0, -dx), min(w, w - dx)
            near_blocked[yd0:yd1, xd0:xd1] |= blocked[ys0:ys1, xs0:xs1]
    # The disc must also stay on the map.
    edge = np.zeros_like(blocked)
    edge[:r, :] = edge[-r:, :] = True
    edge[:, :r] = edge[:, -r:] = True
    return free & ~near_blocked & ~edge


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("maps", nargs="+")
    args = ap.parse_args()

    for path in args.maps:
        meta, free, occupied, unknown = read_map(path)
        res = meta["resolution"]
        cell = res * res
        free_area = free.sum() * cell
        print(f"\n{'=' * 78}")
        print(f" {os.path.basename(path)}")
        print(f"{'=' * 78}")
        print(f"  {free.shape[1]} x {free.shape[0]} cells at {res} m   "
              f"free {free_area:.1f} m2   "
              f"unknown {unknown.sum() * cell:.1f} m2   "
              f"occupied {occupied.sum() * cell:.1f} m2")

        blocked = occupied | unknown
        for name, pts in FOOTPRINTS.items():
            print(f"\n  {name}")
            for pivot_name, pivot_x in (("body centre (base_footprint)", 0.0),
                                        ("drive axle (the real pivot)", DRIVE_AXLE_X)):
                r = circumscribed(pts, pivot_x)
                ok = rotatable(free, blocked, r, res)
                area = ok.sum() * cell
                pct = 100 * area / free_area if free_area else 0
                print(f"      {pivot_name:30s} r {r:.3f} m  "
                      f"needs {2*r:.2f} m wide   "
                      f"turnable {area:7.1f} m2 = {pct:5.1f} % of free")


if __name__ == "__main__":
    sys.exit(main())
