#!/usr/bin/env python3
"""Choose which local controller vica_nav2_params.yaml runs.

    ros2 run vica_description select_controller dwb
    ros2 run vica_description select_controller mppi
    ros2 run vica_description select_controller --show
    ros2 run vica_description select_controller mppi --inflation 0.66

--inflation sets inflation_radius on both costmaps at once. It lives here
rather than in a second script for the reason given below: the comparison is
only readable if one file holds every variable. Both costmaps must move
together -- the planner reads the global one and the controller the local one,
and splitting them produces a robot whose two halves disagree about how wide
the corridor is.

Why it is a variable at all: at 0.55 the inflation is shorter than the padded
circumscribed radius, and nav2 says so out loud at startup --

    [computeCircumscribedCost] The inflation radius (0.550000) is smaller than
    the circumscribed radius (0.650577) ... it cannot use costmap potential
    field to speed up collision checking ... This may significantly slow down
    planning times!

Both planner_server and controller_server log it, every planner cycle.

0.650577 is the handle corner. nav2 pads per axis rather than radially --
sign0(x)*pad on each coordinate -- so (-0.595, -0.035) becomes (-0.645,
-0.085), and hypot of that is the number in the log. Padding radially instead
gives 0.6460, which is wrong by 4.6 mm and in the optimistic direction.
Inscribed comes out 0.2775, not the 0.2574 a radial pad would give.

What nav2 does about the shortfall is set possible_collision_cost_ = 0, and
both consumers read that as "no fast path available" and fall through to the
full polygon check on every scored pose:

    smac    if (possible_collision_cost_ > 0 && cost < it) return free
    MPPI    if (consider_footprint_ && (cost >= it || it < 1.0)) -> footprint

So the effect is slower, not blinder -- but the MPPI branch only holds while
consider_footprint is true. Set it false and the polygon is never checked: all
that is read is the centre cell, and a centre cell 0.60 m from a wall reads 0
under a 0.55 inflation while the handle at 0.6506 m is already in the wall.
This config sets consider_footprint true explicitly. The robot's does not set
it at all.

The file carries both, one named FollowPath and the other FollowPathAlt, and
nav2 loads whichever is called FollowPath. This swaps the two names. Same
approach the planner blocks already use here, and the reason is the same: an
A/B comparison is only worth reading if everything except the one variable is
identical, and two parameter files drift.

It also moves the velocity smoother, which is not a detail. The robot's own
notes record a collision caused by leaving it behind: the smoother was at
max_decel -1.0 while DWB planned against decel_lim_x -2.5, so DWB chose
trajectories believing it could stop in 0.104 s when it really took 0.26 s. The
front right bumper hit something at a commanded 0.010 m/s and an actual 0.227.

The rule the robot settled on is that the smoother sits above the controller as
a loose net, never below it. So the values move with the controller:

    MPPI   ax_max 2.0, az_max 2.0                 ->  accel [2.0, 0, 2.0]
                                                      decel [-2.0, 0, -2.0]
    DWB    acc_lim 2.0/2.0, decel_lim -2.5/-3.2   ->  dev's [2.5, 0, 3.2]
                                                      dev's [-1.0, 0, -3.2]

[미검증] That last line is dev's value and it is the one the collision note
argues against. The deployed config on the robot carries -2.5 for x with the
analysis written out; dev does not have that change. dev is the stated
reference here, so dev's number is used, but a DWB run in simulation is
therefore braking to a limit the running robot no longer uses. Worth settling
before any result from it is carried back.
"""

import argparse
import os
import re
import sys

SETTINGS = {
    "mppi": {
        "plugin": "nav2_mppi_controller::MPPIController",
        "max_accel": "[2.0, 0.0, 2.0]",
        "max_decel": "[-2.0, 0.0, -2.0]",
    },
    "dwb": {
        "plugin": "dwb_core::DWBLocalPlanner",
        "max_accel": "[2.5, 0.0, 3.2]",
        "max_decel": "[-1.0, 0.0, -3.2]",
    },
}

ACTIVE = "    FollowPath:"
INACTIVE = "    FollowPathAlt:"

# Padded circumscribed radius, by nav2's own arithmetic. Below this the
# potential-field fast path is unavailable; nav2 logs an error and every pose
# takes the full polygon check.
CIRCUMSCRIBED = 0.6506
INFLATION_RE = re.compile(r"^(\s*inflation_radius:\s*)([0-9.]+)", re.M)


def current_inflation(text):
    """Both costmaps' inflation_radius, or the single value if they agree."""
    vals = [float(m.group(2)) for m in INFLATION_RE.finditer(text)]
    if not vals:
        raise SystemExit("inflation_radius not found -- is the config intact?")
    return vals


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


def current(text):
    for name, header in (("active", ACTIVE), ("inactive", INACTIVE)):
        if header not in text:
            raise SystemExit(f"{header.strip()} not found -- is the config intact?")
    body = block_of(text, ACTIVE)
    for key, cfg in SETTINGS.items():
        if cfg["plugin"] in body.split("\n")[1]:
            return key
    for key, cfg in SETTINGS.items():
        if cfg["plugin"] in body:
            return key
    raise SystemExit("FollowPath names a plugin this script does not know")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("controller", nargs="?", choices=sorted(SETTINGS),
                    help="which controller nav2 should load")
    ap.add_argument("--config", default=None)
    ap.add_argument("--show", action="store_true", help="report and change nothing")
    ap.add_argument("--inflation", type=float, default=None,
                    help="inflation_radius for both costmaps, metres")
    args = ap.parse_args()

    path = args.config or default_config()
    text = open(path).read()
    now = current(text)
    infl = current_inflation(text)

    def report():
        smoother = re.search(r"max_accel: (\[[^\]]*\])", text)
        decel = re.search(r"max_decel: (\[[^\]]*\])", text)
        shown = f"{infl[0]}" if len(set(infl)) == 1 else f"불일치 {infl}"
        gap = "" if infl[0] >= CIRCUMSCRIBED else \
            f"  <- 외접 {CIRCUMSCRIBED} 미달, fast path 없음"
        print(f"  {os.path.basename(path)}")
        print(f"  현재 컨트롤러 : {now}  ({SETTINGS[now]['plugin']})")
        print(f"  smoother      : accel {smoother.group(1) if smoother else '?'}   "
              f"decel {decel.group(1) if decel else '?'}")
        print(f"  inflation     : {shown}{gap}")

    if args.show or (not args.controller and args.inflation is None):
        report()
        # --show reports and changes nothing, whatever else is on the line.
        # It used to return only when no controller and no --inflation were
        # given, so `--show mppi` printed the current state and then swapped
        # the controller underneath the person reading it.
        if args.show or (not args.controller and args.inflation is None):
            return 0

    changed = []

    if args.controller and args.controller != now:
        # Swap the two block names. Done through a placeholder so the second
        # replacement cannot undo the first.
        text = text.replace(ACTIVE, "    __SWAP__:", 1)
        text = text.replace(INACTIVE, ACTIVE, 1)
        text = text.replace("    __SWAP__:", INACTIVE, 1)

        cfg = SETTINGS[args.controller]
        text = re.sub(r"max_accel: \[[^\]]*\]", f"max_accel: {cfg['max_accel']}", text)
        text = re.sub(r"max_decel: \[[^\]]*\]", f"max_decel: {cfg['max_decel']}", text)
        changed.append(f"컨트롤러 {now} -> {args.controller}   "
                       f"smoother accel {cfg['max_accel']} decel {cfg['max_decel']}")
    elif args.controller:
        print(f"  컨트롤러는 이미 {now} 입니다.")

    if args.inflation is not None:
        # Every occurrence, so the two costmaps cannot diverge. A run where the
        # planner and the controller disagree about corridor width is not a
        # measurement of anything.
        text, n = INFLATION_RE.subn(
            lambda m: f"{m.group(1)}{args.inflation}", text)
        if n < 2:
            raise SystemExit(f"inflation_radius 를 {n}곳만 찾았습니다 -- "
                             "costmap 두 개 모두에 있어야 합니다")
        note = "" if args.inflation >= CIRCUMSCRIBED else \
            f"  [경고] 외접 {CIRCUMSCRIBED} 미달 -- fast path 없음"
        changed.append(f"inflation {infl[0]} -> {args.inflation}  ({n}곳){note}")

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
