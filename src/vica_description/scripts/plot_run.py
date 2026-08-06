#!/usr/bin/env python3
"""Draw where the robot actually went, on top of the course it went through.

    ros2 run vica_description plot_run --stage <stage.usd> --duration 120
    ros2 run vica_description plot_run --stage <stage.usd> --load a.json b.json

[TEST ONLY] Reads /odom and writes files. Publishes nothing and commands
nothing, so it is safe to run beside anything, including on the robot.

Headless simulation gives numbers and no picture, and some mistakes are only
visible as a picture. This course had its wall thickness wrong twice -- lanes
labelled 0.60 m that were really 0.45 -- and both times the arithmetic looked
right and a top-down drawing would have shown it immediately.

Two things it answers that a table does not:

    where did it stop        a run that fails at 0.90 m fails somewhere, and
                             whether that is the corner or the approach is the
                             difference between a controller problem and a
                             planner one
    how repeatable is it     --load overlays several runs. Three paths lying on
                             top of each other and three paths scattered across
                             the corridor are the same table row and not the
                             same result

Geometry comes from the stage file, so the drawing is of the course as built
rather than as intended. Only axis-aligned boxes standing above the floor are
drawn, which is what these courses are made of; a stage built from meshes would
need its own reader.

Odometry is relative to wherever the robot started, so the robot prim's own
translate in the stage is added back to put the path in world coordinates. That
assumes the robot was spawned unrotated, which every course builder here does.
"""

import argparse
import json
import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

# Hangul in the labels renders as empty boxes on the default font. Noto Sans CJK
# ships with the Korean locale on this machine and covers it; the fallback keeps
# the drawing readable rather than correct if it is ever missing.
for _family in ("Noto Sans CJK JP", "Noto Sans CJK KR", "NanumGothic"):
    if any(f.name == _family for f in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _family
        break
matplotlib.rcParams["axes.unicode_minus"] = False

# The nav2 footprint, so the drawing shows the shape that has to fit rather
# than a dot that always does.
FOOTPRINT = [
    (0.305, 0.2275), (0.305, -0.2275), (-0.305, -0.2275),
    (-0.595, -0.035), (-0.595, 0.035), (-0.305, 0.2275),
]
FOOTPRINT_PADDING = 0.05

# Boxes whose top is below this are floor, not wall.
WALL_MIN_TOP = 0.15

# The robot is made of boxes too, and they stand well above the floor line.
#
# Without this exclusion its own chassis, mast and wheels are read as walls and
# baked into the map at the spawn, which is a robot-shaped obstacle exactly
# where the robot is. The planner then answers "Start occupied" to every goal in
# 0.1 seconds, and the run looks like a controller that cannot leave the start.
ROBOT_SUBTREE = "/World/VICA"


def read_course(stage_path):
    """Axis-aligned wall boxes and the robot's spawn, from the stage."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"could not open {stage_path}")
    walls, spawn = [], (0.0, 0.0)
    for prim in stage.Traverse():
        xf = UsdGeom.Xformable(prim)
        if not xf:
            continue
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        if prim.GetName() == "VICA":
            spawn = (t[0], t[1])
            continue
        # Skip the robot's own boxes. Drawn as walls they put a robot-shaped
        # obstacle on the course at the spawn, which is also how they got into
        # the map and made every goal fail with "Start occupied".
        if str(prim.GetPath()).startswith(ROBOT_SUBTREE):
            continue
        if not prim.IsA(UsdGeom.Cube):
            continue
        # Cube size 1 scaled by the transform's diagonal gives the extents.
        sx = m.GetRow3(0).GetLength()
        sy = m.GetRow3(1).GetLength()
        sz = m.GetRow3(2).GetLength()
        size = UsdGeom.Cube(prim).GetSizeAttr().Get() or 1.0
        w, h, d = sx * size, sy * size, sz * size
        if t[2] + d / 2 < WALL_MIN_TOP:
            continue
        walls.append((t[0] - w / 2, t[1] - h / 2, w, h))
    return walls, spawn


def record(duration, odom_topic):
    """Collect the pose track from /odom, in odom coordinates."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    rclpy.init()
    node = Node("plot_run")
    got = []
    node.create_subscription(Odometry, odom_topic, got.append, 50)
    print(f"  {odom_topic} 기록 중, {duration:.0f}초. 그동안 로봇을 주행시키세요.",
          flush=True)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    track = []
    for m in got:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        track.append([
            m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            p.x, p.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)),
        ])
    rclpy.shutdown()
    return track


def footprint_at(x, y, yaw, pad=FOOTPRINT_PADDING):
    """The footprint polygon placed at a pose, grown by the padding."""
    pts = []
    for fx, fy in FOOTPRINT:
        n = math.hypot(fx, fy) or 1.0
        gx, gy = fx + pad * fx / n, fy + pad * fy / n
        pts.append((x + gx * math.cos(yaw) - gy * math.sin(yaw),
                    y + gx * math.sin(yaw) + gy * math.cos(yaw)))
    return pts


def draw(walls, spawn, tracks, labels, out_path, footprint_every):
    fig, ax = plt.subplots(figsize=(14, 9))
    for x, y, w, h in walls:
        ax.add_patch(Rectangle((x, y), w, h, facecolor="#3c4450",
                               edgecolor="none", zorder=1))

    colours = ["#2f7fd8", "#d8542f", "#2fa86b", "#8a4fd8", "#c9a227", "#d82f8a"]
    for i, (track, label) in enumerate(zip(tracks, labels)):
        if not track:
            continue
        c = colours[i % len(colours)]
        xs = [spawn[0] + p[1] for p in track]
        ys = [spawn[1] + p[2] for p in track]
        ax.plot(xs, ys, color=c, linewidth=1.8, zorder=4,
                label=f"{label}  ({len(track)}표본)")
        if footprint_every > 0:
            for k in range(0, len(track), footprint_every):
                ax.add_patch(Polygon(
                    footprint_at(xs[k], ys[k], track[k][3]),
                    closed=True, facecolor="none", edgecolor=c,
                    alpha=0.35, linewidth=0.7, zorder=3))
        ax.plot(xs[0], ys[0], "o", color=c, markersize=9, zorder=5)
        ax.plot(xs[-1], ys[-1], "s", color=c, markersize=9, zorder=5)

    # add_patch does not grow the data limits, so without this the axes stay at
    # the default 0..1 and the drawing comes out empty with everything in it.
    if walls:
        ax.set_xlim(min(x for x, _, _, _ in walls) - 0.5,
                    max(x + w for x, _, w, _ in walls) + 0.5)
        ax.set_ylim(min(y for _, y, _, _ in walls) - 0.5,
                    max(y + h for _, y, _, h in walls) + 0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.2, zorder=0)
    if tracks:
        ax.legend(loc="upper left", fontsize=9)
    ax.set_title("원 = 출발, 사각 = 종료.  얇은 다각형은 padding 포함 footprint")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print(f"  wrote {out_path}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", required=True, help="course .usd to draw under the path")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--out", default="run.png")
    ap.add_argument("--save", default=None,
                    help="also write the raw track here, for later --load")
    ap.add_argument("--load", nargs="+", default=None,
                    help="draw saved tracks instead of recording")
    ap.add_argument("--course-only", action="store_true",
                    help="draw the course and stop, without recording anything")
    ap.add_argument("--footprint-every", type=int, default=25,
                    help="draw the footprint every Nth sample; 0 for none")
    args = ap.parse_args()

    walls, spawn = read_course(args.stage)
    print(f"  코스 {os.path.basename(args.stage)}: 벽 {len(walls)}개, "
          f"로봇 출발 ({spawn[0]:+.2f}, {spawn[1]:+.2f})", flush=True)

    if args.course_only:
        draw(walls, spawn, [], [], args.out, 0)
        return 0

    if args.load:
        tracks, labels = [], []
        for path in args.load:
            with open(path) as fh:
                tracks.append(json.load(fh))
            labels.append(os.path.splitext(os.path.basename(path))[0])
    else:
        track = record(args.duration, args.odom)
        if len(track) < 5:
            print(f"  표본 {len(track)}개 -- {args.odom} 가 발행되고 있나요?",
                  file=sys.stderr)
            return 1
        tracks, labels = [track], ["run"]
        if args.save:
            with open(args.save, "w") as fh:
                json.dump(track, fh)
            print(f"  wrote {args.save}", flush=True)

    draw(walls, spawn, tracks, labels, args.out, args.footprint_every)
    return 0


if __name__ == "__main__":
    sys.exit(main())
