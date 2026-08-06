#!/usr/bin/env python3
"""Choose which local controller vica_nav2_params.yaml runs.

    ros2 run vica_description select_controller dwb
    ros2 run vica_description select_controller mppi
    ros2 run vica_description select_controller --show

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
    args = ap.parse_args()

    path = args.config or default_config()
    text = open(path).read()
    now = current(text)

    if args.show or not args.controller:
        smoother = re.search(r"max_accel: (\[[^\]]*\])", text)
        decel = re.search(r"max_decel: (\[[^\]]*\])", text)
        print(f"  {os.path.basename(path)}")
        print(f"  현재 컨트롤러 : {now}  ({SETTINGS[now]['plugin']})")
        print(f"  smoother      : accel {smoother.group(1) if smoother else '?'}   "
              f"decel {decel.group(1) if decel else '?'}")
        if not args.controller:
            return 0

    if args.controller == now:
        print(f"  이미 {now} 입니다. 변경 없음.")
        return 0

    # Swap the two block names. Done through a placeholder so the second
    # replacement cannot undo the first.
    text = text.replace(ACTIVE, "    __SWAP__:", 1)
    text = text.replace(INACTIVE, ACTIVE, 1)
    text = text.replace("    __SWAP__:", INACTIVE, 1)

    cfg = SETTINGS[args.controller]
    text = re.sub(r"max_accel: \[[^\]]*\]", f"max_accel: {cfg['max_accel']}", text)
    text = re.sub(r"max_decel: \[[^\]]*\]", f"max_decel: {cfg['max_decel']}", text)

    open(path, "w").write(text)
    print(f"  {now} -> {args.controller}")
    print(f"  smoother accel {cfg['max_accel']}  decel {cfg['max_decel']}")
    print("  colcon build 후 nav2 를 다시 띄워야 반영됩니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
