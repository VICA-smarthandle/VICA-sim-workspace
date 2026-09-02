#!/usr/bin/env python3
"""Choose which local controller and global planner vica_nav2_params.yaml runs.

    ros2 run vica_description select_controller dwb
    ros2 run vica_description select_controller mppi --planner hybrid
    ros2 run vica_description select_controller --planner navfn
    ros2 run vica_description select_controller --show
    ros2 run vica_description select_controller rpp --inflation 0.66

Controllers: dwb, rpp, mppi          Planners: lattice, hybrid, navfn, smac2d

How the swap works
------------------
The behaviour tree asks for its controller by name:

    <ControllerSelector default_controller="FollowPath" .../>
    <PlannerSelector    default_planner="GridBased"    .../>

so the loaded blocks have to be called exactly that. Every other block is
inert -- nav2 reads only the key named in controller_plugins/planner_plugins.
So the config carries all seven blocks at once, six of them under identity
names (FollowPathRPP, GridBasedHybrid, ...), and this script renames the pair
being selected to FollowPath/GridBased and the outgoing pair back to its
identity name.

Which one is currently active is read from its plugin string, not remembered
in a file. Each of the seven plugins is distinct, so the config is
self-describing and a hand edit cannot desynchronise it from a state file.

Why one script for both axes, and for inflation
-----------------------------------------------
A comparison is only readable if one file holds every variable, and these
three move together more often than not:

  - Both costmaps' inflation_radius must move as one. The planner reads the
    global and the controller the local; split them and the robot's two halves
    disagree about how wide the corridor is.

  - The velocity smoother moves with the controller. The robot's notes record
    a collision caused by leaving it behind: the smoother was at max_decel
    -1.0 while DWB planned against decel_lim_x -2.5, so DWB chose trajectories
    believing it could stop in 0.104 s when it really took 0.26 s. The front
    right bumper hit something at a commanded 0.010 m/s and an actual 0.227.
    The rule the robot settled on is that the smoother sits above the
    controller as a loose net, never below it.

    MPPI   ax_max 2.0, az_max 2.0                ->  accel [2.0, 0, 2.0]
                                                     decel [-2.0, 0, -2.0]
    DWB    acc_lim 2.0/2.0, decel_lim -1.5/-3.2  ->  dev's [2.5, 0, 3.2]
                                                     dev's [-1.25, 0, -3.2]
    RPP    max_angular_accel 2.0, no linear      ->  DWB's, as the looser net

On inflation_radius
-------------------
nav2 wants inflation_radius >= the padded circumscribed radius. Below it,
possible_collision_cost_ is set to 0 and both consumers read that as "no fast
path available", falling through to the full polygon check on every scored
pose:

    smac    if (possible_collision_cost_ > 0 && cost < it) return free
    MPPI    if (consider_footprint_ && (cost >= it || it < 1.0)) -> footprint

So the effect is slower, not blinder -- but the MPPI branch only holds while
consider_footprint is true. Set it false and the polygon is never checked.
This config sets it true explicitly; the robot's does not set it at all.

Since 2026-09-02 the footprint matches the robot's shortened body and the
padded circumscribed radius is 0.4727, so the 0.55 inflation clears it. It did
not before: at 0.6506 nav2 logged the shortfall every planner cycle.
"""

import argparse
import math
import os
import re
import sys

CONTROLLERS = {
    "dwb": {
        "block": "FollowPathDWB",
        "plugin": "dwb_core::DWBLocalPlanner",
        "max_accel": "[2.5, 0.0, 3.2]",
        "max_decel": "[-1.25, 0.0, -3.2]",
    },
    "rpp": {
        "block": "FollowPathRPP",
        "plugin": "nav2_regulated_pure_pursuit_controller::"
                  "RegulatedPurePursuitController",
        "max_accel": "[2.5, 0.0, 3.2]",
        "max_decel": "[-1.25, 0.0, -3.2]",
    },
    "mppi": {
        "block": "FollowPathMPPI",
        "plugin": "nav2_mppi_controller::MPPIController",
        "max_accel": "[2.0, 0.0, 2.0]",
        "max_decel": "[-2.0, 0.0, -2.0]",
    },
}

PLANNERS = {
    "lattice": {
        "block": "GridBasedLattice",
        "plugin": "nav2_smac_planner::SmacPlannerLattice",
    },
    "hybrid": {
        "block": "GridBasedHybrid",
        "plugin": "nav2_smac_planner::SmacPlannerHybrid",
    },
    "navfn": {
        "block": "GridBasedNavFn",
        "plugin": "nav2_navfn_planner::NavfnPlanner",
    },
    "smac2d": {
        "block": "GridBased2D",
        "plugin": "nav2_smac_planner::SmacPlanner2D",
    },
}

ACTIVE_CONTROLLER = "FollowPath"
ACTIVE_PLANNER = "GridBased"

# The footprint this config carries, and its padding. The circumscribed radius
# is derived rather than written down so that editing the footprint cannot
# leave a stale number here -- which is what the 0.6506 in this file's previous
# version became the moment the body was shortened.
FOOTPRINT_RE = re.compile(r'^\s*footprint:\s*"(\[\[.*?\]\])"', re.M)
PADDING_RE = re.compile(r"^\s*footprint_padding:\s*([0-9.]+)", re.M)
INFLATION_RE = re.compile(r"^(\s*inflation_radius:\s*)([0-9.]+)", re.M)


def circumscribed(text):
    """Padded circumscribed radius, by nav2's own arithmetic.

    nav2 pads per axis -- sign(x)*pad on each coordinate -- not radially.
    Padding radially instead is optimistic by several millimetres, which is
    the wrong direction for a clearance number.
    """
    m = FOOTPRINT_RE.search(text)
    p = PADDING_RE.search(text)
    if not m or not p:
        return None
    pad = float(p.group(1))
    pts = re.findall(r"\[\s*(-?[0-9.]+)\s*,\s*(-?[0-9.]+)\s*\]", m.group(1))
    best = 0.0
    for xs, ys in pts:
        x, y = float(xs), float(ys)
        x += math.copysign(pad, x) if x else pad
        y += math.copysign(pad, y) if y else pad
        best = max(best, math.hypot(x, y))
    return round(best, 4)


def default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("vica_description"),
                         "config", "vica_nav2_params.yaml")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), "config", "vica_nav2_params.yaml")


def block_of(text, header):
    """The text of the block introduced by `header`, up to the next block."""
    i = text.index(header)
    j = text.find("\n    ", i + len(header))
    while j != -1 and text[j + 5:j + 6] in (" ", "#"):
        j = text.find("\n    ", j + 1)
    return text[i:j if j != -1 else len(text)]


def current(text, active_name, table, what):
    """Which entry of `table` the block named `active_name` holds."""
    header = f"    {active_name}:"
    if header not in text:
        raise SystemExit(f"{active_name} 블록이 없습니다 -- 설정이 온전한가요?")
    body = block_of(text, header)
    for key, cfg in table.items():
        if cfg["plugin"] in body:
            return key
    raise SystemExit(f"{active_name} 가 이 스크립트가 모르는 {what} 를 가리킵니다")


def swap(text, active_name, table, now, want):
    """Rename `want`'s block to active_name and active_name's back to `now`'s.

    Through a placeholder, so the second rename cannot undo the first.
    """
    outgoing = f"    {table[now]['block']}:"
    incoming = f"    {table[want]['block']}:"
    if incoming not in text:
        raise SystemExit(f"{table[want]['block']} 블록이 없습니다")
    text = text.replace(f"    {active_name}:", "    __SWAP__:", 1)
    text = text.replace(incoming, f"    {active_name}:", 1)
    return text.replace("    __SWAP__:", outgoing, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("controller", nargs="?", choices=sorted(CONTROLLERS),
                    help="which local controller nav2 should load")
    ap.add_argument("--planner", choices=sorted(PLANNERS), default=None,
                    help="which global planner nav2 should load")
    ap.add_argument("--config", default=None)
    ap.add_argument("--show", action="store_true", help="report and change nothing")
    ap.add_argument("--inflation", type=float, default=None,
                    help="inflation_radius for both costmaps, metres")
    args = ap.parse_args()

    path = args.config or default_config()
    text = open(path).read()
    now_c = current(text, ACTIVE_CONTROLLER, CONTROLLERS, "controller")
    now_p = current(text, ACTIVE_PLANNER, PLANNERS, "planner")
    infl = [float(m.group(2)) for m in INFLATION_RE.finditer(text)]
    if not infl:
        raise SystemExit("inflation_radius 를 찾지 못했습니다")
    circ = circumscribed(text)

    def report():
        accel = re.search(r"max_accel: (\[[^\]]*\])", text)
        decel = re.search(r"max_decel: (\[[^\]]*\])", text)
        shown = f"{infl[0]}" if len(set(infl)) == 1 else f"불일치 {infl}"
        gap = ""
        if circ is not None and infl[0] < circ:
            gap = f"  <- 외접 {circ} 미달, fast path 없음"
        print(f"  {os.path.basename(path)}")
        print(f"  controller : {now_c:8s} {CONTROLLERS[now_c]['plugin']}")
        print(f"  planner    : {now_p:8s} {PLANNERS[now_p]['plugin']}")
        print(f"  smoother   : accel {accel.group(1) if accel else '?'}   "
              f"decel {decel.group(1) if decel else '?'}")
        print(f"  inflation  : {shown}   외접 {circ}{gap}")

    if args.show or (not args.controller and args.planner is None
                     and args.inflation is None):
        report()
        return 0

    changed = []

    if args.controller and args.controller != now_c:
        text = swap(text, ACTIVE_CONTROLLER, CONTROLLERS, now_c, args.controller)
        cfg = CONTROLLERS[args.controller]
        text = re.sub(r"max_accel: \[[^\]]*\]", f"max_accel: {cfg['max_accel']}", text)
        text = re.sub(r"max_decel: \[[^\]]*\]", f"max_decel: {cfg['max_decel']}", text)
        changed.append(f"controller {now_c} -> {args.controller}   "
                       f"smoother accel {cfg['max_accel']} decel {cfg['max_decel']}")
    elif args.controller:
        print(f"  controller 는 이미 {now_c} 입니다.")

    if args.planner and args.planner != now_p:
        text = swap(text, ACTIVE_PLANNER, PLANNERS, now_p, args.planner)
        changed.append(f"planner    {now_p} -> {args.planner}")
    elif args.planner:
        print(f"  planner 는 이미 {now_p} 입니다.")

    if args.inflation is not None:
        # Every occurrence, so the two costmaps cannot diverge. A run where the
        # planner and the controller disagree about corridor width is not a
        # measurement of anything.
        text, n = INFLATION_RE.subn(
            lambda m: f"{m.group(1)}{args.inflation}", text)
        if n < 2:
            raise SystemExit(f"inflation_radius 를 {n}곳만 찾았습니다 -- "
                             "costmap 두 개 모두에 있어야 합니다")
        note = ""
        if circ is not None and args.inflation < circ:
            note = f"  [경고] 외접 {circ} 미달 -- fast path 없음"
        changed.append(f"inflation  {infl[0]} -> {args.inflation}  ({n}곳){note}")

    if not changed:
        print("  변경 없음.")
        return 0

    open(path, "w").write(text)
    for line in changed:
        print(f"  {line}")
    print("  colcon build 후 nav2 를 다시 띄워야 반영됩니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
