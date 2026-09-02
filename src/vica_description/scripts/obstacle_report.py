#!/usr/bin/env python3
"""Turn a dynamic-obstacle trial into the three distances a safety case needs.

    obstacle_report.py <results_dir>
    obstacle_report.py <trial.csv> --walk <walk.csv>

Reads two logs and joins them:

    trial csv   what the robot knew and was told to do -- odom, cmd_vel, the
                nearest return in a forward cone, both ultrasonic ranges.
                Written by obstacle_trial.py, from ROS.
    walk csv    where the walker actually was, and how fast the robot actually
                went. Written by play_stage.py, from inside the simulator.

Joined on the robot's own x rather than on a timestamp. Both files carry it,
the approach is monotonic in it, and the two clocks have different origins --
one counts frames from the start of the play loop, the other reads /odom header
stamps. Matching on the quantity both agree about removes a whole class of
alignment bug that would show up as a plausible-looking wrong answer.

The three numbers
-----------------
    감지거리        gap between robot and walker at the first sample where any
                    sensor has it: the forward scan return drops below the
                    clear-corridor baseline, or an ultrasonic reads short of
                    its own max_range
    감속 시작거리   gap when the commanded forward speed has fallen 10 % below
                    the cruise value it held on the approach
    정지거리        how far the robot travelled between that moment and coming
                    to rest, or to its slowest, if it never stopped

and two more that decide whether the numbers were enough:

    최근접          smallest gap reached
    결과            반응없음 / 감속 / 정지 / 접촉
"""

import argparse
import bisect
import csv
import glob
import json
import math
import os
import re
import sys

# The robot's own outline, unpadded, in its own frame. From the URDF: the
# footprint hexagon runs x -0.415 to +0.305 and y -0.225 to +0.225, and a
# rectangle through those bounds is what this measures against.
#
# Not a circle. A circumscribed radius of 0.31 is right for something the robot
# is driving at and wrong for something it is passing beside, and it is the
# passing case that decides whether squeezing past a person was safe. Using the
# circle reported a 0.23 m contact on a run where the robot's own trajectory
# put its side 0.155 m clear of the walker.
# The nav2 footprint itself, unpadded, from config/vica_nav2_params.yaml. A
# rectangle through its bounds was the first attempt and it is wrong where it
# matters: the hexagon's rear is cut back to +/-0.035 at x -0.415 and only
# reaches +/-0.225 at x -0.305, and the corner that grazes a person the robot
# is turning away from is exactly that cut one. The rectangle over-reported the
# overlap by 0.10 m.
ROBOT_FOOTPRINT = [
    (0.305, 0.225), (0.305, -0.225), (-0.305, -0.225),
    (-0.415, -0.035), (-0.415, 0.035), (-0.305, 0.225),
]
# Kept for the detection test, which is about how far away something is seen
# and not about which part of the robot is nearest.
ROBOT_R = 0.31
STOPPED = 0.03          # m/s, below which it is stopped rather than crawling
DECEL_FRAC = 0.90       # of cruise speed
CLEAR_MARGIN = 0.25     # m the forward scan must shorten by to count as a sight
# Closest approach above which driving straight past was the right answer.
# 0.6 m is roughly the robot's own length: at that separation the walker was
# never on the robot's line and nothing was owed.
CLEAR_PASS = 0.60


def _read(path):
    with open(path) as fh:
        return [{k: (float(v) if v not in ("", None) else float("nan"))
                 for k, v in row.items()} for row in csv.DictReader(fh)]


def _point_in_polygon(px, py, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            xc = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < xc:
                inside = not inside
    return inside


def _dist_to_segment(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    n2 = dx * dx + dy * dy
    t = 0.0 if n2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / n2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def _clearance(rx, ry, yaw, wx, wy, wr):
    """Gap between the robot's outline and the walker's, in metres.

    Negative means they overlap. The walker is put into the robot's frame and
    measured against the footprint polygon, so passing beside someone is scored
    on the robot's width, driving at them on its length, and turning away from
    them on whichever corner is actually nearest -- which is the case that
    matters here, because what grazes a person is the tail swinging out.
    """
    dx, dy = wx - rx, wy - ry
    c, s = math.cos(-yaw), math.sin(-yaw)
    px, py = dx * c - dy * s, dx * s + dy * c
    n = len(ROBOT_FOOTPRINT)
    d = min(_dist_to_segment(px, py, *ROBOT_FOOTPRINT[i],
                             *ROBOT_FOOTPRINT[(i + 1) % n]) for i in range(n))
    if _point_in_polygon(px, py, ROBOT_FOOTPRINT):
        d = -d
    return d - wr


class _ByTime:
    """Walk rows, findable by the trial's clock.

    Joined on time with one offset, not on the robot's x. Matching on x was the
    first attempt and it is wrong in exactly the case the trial is about: a
    robot that stops in front of the walker has hundreds of samples at the same
    x, and the lookup returns whichever of them the sort happened to put first,
    which is usually one from before the walker moved. That produced a closest
    approach of 1.47 m in a 2.00 m corridor with the walker standing in the
    middle of it -- a number that cannot happen.

    The two logs count from different origins: one counts frames from the start
    of the play loop, the other reads /odom header stamps. The offset between
    them is constant, and recoverable from the one event both files agree on,
    which is the robot first moving.
    """

    def __init__(self, walk, trial):
        self.rows = sorted(walk, key=lambda w: w["t"])
        self.keys = [w["t"] for w in self.rows]
        self.offset = self._align(walk, trial)

    @staticmethod
    def _align(walk, trial):
        """Trial time minus walk time, from when each log first shows motion."""
        def first_move(rows, key):
            x0 = rows[0][key]
            for r in rows:
                if abs(r[key] - x0) > 0.20:
                    return r["t"]
            return None
        a = first_move(trial, "x")
        b = first_move(walk, "robot_x")
        return (a - b) if (a is not None and b is not None) else 0.0

    def at(self, t):
        if not self.rows:
            return None
        i = bisect.bisect_left(self.keys, t - self.offset)
        best, bd = None, float("inf")
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(self.rows):
                d = abs(self.keys[j] - (t - self.offset))
                if d < bd:
                    best, bd = self.rows[j], d
        return best


def analyse(trial_csv, walk_csv, meta):
    trial = _read(trial_csv)
    walk = _read(walk_csv)
    if not trial or not walk:
        return None

    wr = float(meta.get("walker", {}).get("radius", 0.175))
    cross_x = float(meta.get("walker", {}).get("cross_x", 0.0))
    contact = ROBOT_R + wr

    index = _ByTime(walk, trial)
    rows = []
    for s in trial:
        w = index.at(s["t"])
        if w is None:
            continue
        gap = _clearance(s["x"], s["y"], s["yaw"], cross_x, w["walker_y"], wr)
        rows.append({**s, "walker_y": w["walker_y"], "walking": w["walking"],
                     "true_speed": w["robot_speed"], "gap": gap})
    if not rows:
        return None

    walking = [r for r in rows if r["walking"] >= 0.5]
    if not walking:
        return {"note": "보행자가 움직이지 않았습니다"}

    # Cruise speed: the median commanded speed over the approach, before the
    # walker moved. A mean would be dragged down by the start-up ramp.
    before = [r["cmd_vx"] for r in rows if r["walking"] < 0.5 and r["cmd_vx"] > 0.05]
    before.sort()
    cruise = before[len(before) // 2] if before else float("nan")

    def first(pred, seq):
        for r in seq:
            if pred(r):
                return r
        return None

    us_max = 1.50
    # The lidar sits near the robot's centre, so the distance it should read to
    # the walker is the gap plus the robot's own half-diagonal: gap is measured
    # surface to surface and already has both radii taken out of it. Reading
    # shorter than that by CLEAR_MARGIN means something is there that was not
    # there before.
    seen = first(
        lambda r: (r["us_left"] < us_max - 0.01 or r["us_right"] < us_max - 0.01
                   or r["scan_ahead"] < r["gap"] + ROBOT_R + CLEAR_MARGIN),
        walking)
    decel = first(lambda r: r["cmd_vx"] < cruise * DECEL_FRAC, walking) \
        if cruise == cruise else None
    closest = min(walking, key=lambda r: r["gap"])

    stop = None
    if decel is not None:
        after = [r for r in walking if r["t"] >= decel["t"]]
        stop = first(lambda r: r["true_speed"] < STOPPED, after)
        slowest = min(after, key=lambda r: r["true_speed"]) if after else None
    else:
        slowest = None

    # "It did not react" is two different results and they must not share a
    # row. A walker who has finished crossing before the robot arrives needs no
    # reaction, and the robot driving past at full speed is correct; a walker
    # still in the way and no reaction is the failure the trial exists to
    # catch. The closest approach separates them.
    if closest["gap"] <= 0.0:
        verdict = "접촉"
    elif stop is not None:
        verdict = "정지"
    elif decel is not None:
        verdict = "감속"
    elif closest["gap"] >= CLEAR_PASS:
        verdict = "여유통과"
    else:
        verdict = "반응없음"

    travelled = None
    if decel is not None:
        end = stop or slowest
        if end is not None:
            travelled = abs(end["x"] - decel["x"])

    return {
        "outcome": meta.get("outcome"),
        "trigger_m": meta.get("trigger_m"),
        "cruise": cruise,
        "detect_gap": seen["gap"] if seen else None,
        "decel_gap": decel["gap"] if decel else None,
        "stop_travel": travelled,
        "closest": closest["gap"],
        "min_speed": slowest["true_speed"] if slowest else None,
        "verdict": verdict,
    }


def _f(v, w=8, p=3, dash="-"):
    return f"{dash:>{w}}" if v is None or v != v else f"{v:{w}.{p}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="results directory, or one trial csv")
    ap.add_argument("--walk", help="walk csv, when a single trial is given")
    args = ap.parse_args()

    jobs = []
    if os.path.isdir(args.target):
        for csv_path in sorted(glob.glob(os.path.join(args.target, "*.csv"))):
            if csv_path.endswith("_walk.csv"):
                continue
            meta_path = os.path.splitext(csv_path)[0] + ".json"
            walk_path = os.path.splitext(csv_path)[0] + "_walk.csv"
            if os.path.exists(meta_path) and os.path.exists(walk_path):
                jobs.append((csv_path, walk_path, meta_path))
    else:
        meta_path = os.path.splitext(args.target)[0] + ".json"
        jobs.append((args.target, args.walk, meta_path))

    if not jobs:
        print("분석할 시험이 없습니다.")
        return 1

    print()
    print("  동적 장애물 대응 — 보행자가 문에서 나와 복도를 가로지른다")
    print("  " + "=" * 88)
    print(f"  {'출발거리':>8} {'순항v':>7} {'감지거리':>9} {'감속거리':>9} "
          f"{'정지주행':>9} {'최근접':>8} {'최저v':>7}  판정")
    print("  " + "-" * 88)
    rows = []
    for csv_path, walk_path, meta_path in jobs:
        with open(meta_path) as fh:
            meta = json.load(fh)
        # The trigger in the metadata is wrong for every trial written before
        # 2026-09-03: obstacle_sweep sets VICA_WALK_TRIGGER_M for the simulator
        # and obstacle_trial reads it from its own environment, where it is not
        # set, so it recorded the default. The filename is the sweep's own
        # record of what it asked for, so it wins.
        m = re.search(r"trigger_([0-9.]+)_", os.path.basename(csv_path))
        if m:
            meta["trigger_m"] = float(m.group(1))
        r = analyse(csv_path, walk_path, meta)
        if r is None or "note" in r:
            print(f"  {os.path.basename(csv_path)}: "
                  f"{r.get('note') if r else '읽을 수 없음'}")
            continue
        rows.append(r)
        print(f"  {_f(r['trigger_m'], 8, 2)} {_f(r['cruise'], 7)} "
              f"{_f(r['detect_gap'], 9)} {_f(r['decel_gap'], 9)} "
              f"{_f(r['stop_travel'], 9)} {_f(r['closest'], 8)} "
              f"{_f(r['min_speed'], 7)}  {r['verdict']}")
    print("  " + "-" * 88)
    print("  거리는 모두 로봇 외곽과 보행자 외곽 사이의 여유입니다 (0 이면 접촉).")
    print("  출발거리 = 보행자가 걷기 시작한 순간 로봇이 교차점에서 떨어져 있던 거리.")

    ok = [r for r in rows if r["verdict"] in ("정지", "감속") and r["closest"] > 0]
    passed = [r for r in rows if r["verdict"] == "여유통과"]
    if passed:
        latest = min(passed, key=lambda r: r["trigger_m"])
        print()
        print(f"  반응이 필요 없던 가장 늦은 등장: 출발거리 {latest['trigger_m']:.2f} m "
              f"(최근접 {latest['closest']:.3f} m) — 보행자가 먼저 지나갔다")
    if ok:
        worst = min(ok, key=lambda r: r["trigger_m"])
        print()
        print(f"  우회·정지가 성립한 가장 늦은 등장: 출발거리 {worst['trigger_m']:.2f} m "
              f"(최근접 {worst['closest']:.3f} m)")
    bad = [r for r in rows if r["verdict"] == "접촉"]
    if bad:
        latest = max(bad, key=lambda r: r["trigger_m"])
        print(f"  접촉이 일어난 가장 이른 등장: 출발거리 {latest['trigger_m']:.2f} m")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
