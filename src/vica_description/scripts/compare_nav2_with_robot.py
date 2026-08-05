#!/usr/bin/env python3
"""Compare this workspace's Nav2 configuration with the physical robot's.

    ros2 run vica_description compare_nav2_with_robot
    compare_nav2_with_robot.py --robot /path/to/robot/nav2_params.yaml

Exits non-zero when something that must match does not, so it can gate a
commit or a run.

Why it exists
-------------
Two Nav2 configurations describing the same robot drift, and they drift
silently: nothing fails, the robot just behaves differently in simulation than
it does on the floor, and the difference is discovered later as a mystery.

Both directions of copying have gone wrong here already. Copying the robot's
config verbatim brought Humble plugin names into a Jazzy workspace, and
planner_server aborted at configure with every navigation node going down
behind it. Going the other way, an inflation radius was raised in simulation
on reasoning that ignored the corridor widths the robot was measured in.

So the useful question is not "are these the same" -- they should not be -- but
"is each difference one we chose". This sorts them.

Three kinds of parameter
------------------------
MUST_MATCH        Measured on the robot. Geometry, limits, costmap tuning.
                  A difference here is a bug in simulation, and simulation
                  results do not transfer until it is fixed.

SIM_AUTHORITATIVE Established in simulation and not yet applied to the robot.
                  Sensor specifications live here: the A2's ranges were pinned
                  down against the asset and the robot's config still carries a
                  placeholder.

MUST_DIFFER       Deliberately different, with the reason recorded. A match
                  here is as suspicious as a mismatch elsewhere.

Anything not listed is reported as unclassified rather than ignored, because
the list going stale is the way this stops working.
"""

import argparse
import sys

import yaml


DEFAULT_ROBOT = (
    "/home/sim/VICA-smarthandle/vica_ros2_ws/src/vica_nav2/config/nav2_params.yaml"
)
DEFAULT_SIM = "config/vica_nav2_params.yaml"


# (node, parameter path under ros__parameters) -> why it matters
MUST_MATCH = {
    ("local_costmap.local_costmap", "footprint"): "measured hull",
    ("global_costmap.global_costmap", "footprint"): "measured hull",
    ("local_costmap.local_costmap", "footprint_padding"): "measured hull",
    ("global_costmap.global_costmap", "footprint_padding"): "measured hull",
    ("local_costmap.local_costmap", "inflation_layer.inflation_radius"):
        "set against measured corridor widths; 0.70 removes the free centreline "
        "in half of them",
    ("global_costmap.global_costmap", "inflation_layer.inflation_radius"):
        "set against measured corridor widths",
    ("local_costmap.local_costmap", "inflation_layer.cost_scaling_factor"):
        "2.5 measured as decaying too slowly",
    ("global_costmap.global_costmap", "inflation_layer.cost_scaling_factor"):
        "2.5 measured as decaying too slowly",
    ("local_costmap.local_costmap", "resolution"): "matches the lattice primitives",
    ("local_costmap.local_costmap", "width"): "sized to the controller horizon",
    ("local_costmap.local_costmap", "height"): "sized to the controller horizon",
    # Scan gating, decided on the robot against its own lidar.
    ("local_costmap.local_costmap", "voxel_layer.scan.obstacle_min_range"):
        "the robot discards returns closer than this; simulation keeping "
        "everything is why its costmap holds marks the robot's does not",
    ("local_costmap.local_costmap", "voxel_layer.scan.raytrace_min_range"):
        "as obstacle_min_range, for clearing",
    ("local_costmap.local_costmap", "voxel_layer.scan.obstacle_max_range"):
        "how far a return is trusted to mark",
    ("local_costmap.local_costmap", "voxel_layer.scan.raytrace_max_range"):
        "how far a beam is trusted to clear",
    ("local_costmap.local_costmap", "voxel_layer.scan.observation_persistence"):
        "how long a mark survives without being seen again",
    ("local_costmap.local_costmap", "voxel_layer.scan.expected_update_rate"):
        "how long a silent sensor is tolerated before the layer complains",
    ("local_costmap.local_costmap", "voxel_layer.mark_threshold"):
        "voxel column hits needed to mark the cell",
    ("global_costmap.global_costmap", "obstacle_layer.scan.obstacle_min_range"):
        "as the local costmap",
    ("global_costmap.global_costmap", "obstacle_layer.scan.raytrace_min_range"):
        "as the local costmap",
}

SIM_AUTHORITATIVE = {
    ("amcl", "laser_max_range"):
        "RPLIDAR A2 reaches 12 m; the robot still declares 100.0, so its "
        "likelihood field reasons about returns the sensor cannot produce",
    ("amcl", "laser_min_range"):
        "A2 minimum; -1.0 leaves it to the driver",
    ("local_costmap.local_costmap", "voxel_layer.z_voxels"):
        "16 voxels of 0.05 is a 0.80 m ceiling under a max_obstacle_height of "
        "2.0: the layer cannot represent what it is told to look for. 40 makes "
        "the two agree. Both configurations carried the contradiction; this "
        "one no longer does",
}

MUST_DIFFER = {
    ("controller_server", "FollowPath.plugin"):
        "MPPI in simulation for footprint-aware horizons, DWB on the robot",
    ("amcl", "alpha1"): "Isaac odometry is ground truth; 0.2 spreads particles "
                        "for noise that is not there",
    ("amcl", "alpha2"): "as alpha1",
    ("amcl", "alpha3"): "as alpha1",
    ("amcl", "alpha4"): "as alpha1",
    ("amcl", "alpha5"): "as alpha1",
    ("planner_server", "GridBased.plugin"):
        "Jazzy names plugins with ::, Humble with /",
    ("planner_server", "GridBasedAlt.plugin"):
        "as GridBased; kept inert for A/B",
    ("planner_server", "GridBased.lattice_filepath"):
        "primitives ship with the distribution, so the path follows it",
    ("local_costmap.local_costmap", "plugins"):
        "the robot runs an nvblox_layer between voxel and inflation, feeding "
        "D455 depth into the costmap so it sees what a lidar plane at 0.382 m "
        "cannot. Simulation has no equivalent yet -- a gap to close for the "
        "dynamic-obstacle work, not a difference to preserve",
}


def flatten(node, prefix=""):
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            out.update(flatten(v, f"{prefix}{k}." if prefix or True else k))
    else:
        out[prefix.rstrip(".")] = node
    return out


def params_of(doc, node_name):
    """ros__parameters of a node, allowing for nested node names."""
    cur = doc
    for part in node_name.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return {}
        cur = cur[part]
    if not isinstance(cur, dict):
        return {}
    return flatten(cur.get("ros__parameters", {}))


def fmt(v):
    if v is None:
        return "(absent)"
    s = str(v)
    return s if len(s) <= 46 else s[:43] + "..."


def check(sim_doc, robot_doc, table, expect_equal, label):
    rows, bad = [], 0
    for (node, param), why in table.items():
        a = params_of(sim_doc, node).get(param)
        b = params_of(robot_doc, node).get(param)
        equal = a == b
        ok = equal if expect_equal else not equal
        if not ok:
            bad += 1
        rows.append((ok, node, param, a, b, why))

    print(f"\n{'=' * 100}\n{label}\n{'=' * 100}")
    for ok, node, param, a, b, why in rows:
        mark = "  ok " if ok else " >>> "
        print(f"{mark}{node}.{param}")
        print(f"        sim   {fmt(a)}")
        print(f"        robot {fmt(b)}")
        if not ok:
            print(f"        why   {why}")
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sim", default=DEFAULT_SIM)
    ap.add_argument("--robot", default=DEFAULT_ROBOT)
    args = ap.parse_args()

    try:
        sim_doc = yaml.safe_load(open(args.sim))
    except OSError as exc:
        print(f"cannot read sim config: {exc}", file=sys.stderr)
        return 2
    try:
        robot_doc = yaml.safe_load(open(args.robot))
    except OSError as exc:
        print(f"cannot read robot config: {exc}", file=sys.stderr)
        print("Pass --robot if the robot workspace is elsewhere.", file=sys.stderr)
        return 2

    print(f"sim   {args.sim}")
    print(f"robot {args.robot}")

    bad = check(sim_doc, robot_doc, MUST_MATCH, True,
                "MUST MATCH -- measured on the robot; a difference is a bug here")
    bad += check(sim_doc, robot_doc, SIM_AUTHORITATIVE, False,
                 "SIM AUTHORITATIVE -- established here, still to apply to the robot")
    bad += check(sim_doc, robot_doc, MUST_DIFFER, False,
                 "MUST DIFFER -- a match here is as suspicious as a mismatch above")

    # Anything the tables do not cover. Silence here would let the lists rot.
    covered = set(MUST_MATCH) | set(SIM_AUTHORITATIVE) | set(MUST_DIFFER)
    nodes = {n for n, _ in covered}
    unclassified = []
    for node in sorted(nodes):
        a, b = params_of(sim_doc, node), params_of(robot_doc, node)
        for key in sorted(set(a) | set(b)):
            if (node, key) in covered:
                continue
            if a.get(key) != b.get(key):
                unclassified.append((node, key, a.get(key), b.get(key)))

    print(f"\n{'=' * 100}\nUNCLASSIFIED differences in the same nodes ({len(unclassified)})"
          f"\n{'=' * 100}")
    for node, key, a, b in unclassified:
        print(f"  {node}.{key}")
        print(f"        sim   {fmt(a)}")
        print(f"        robot {fmt(b)}")
    if unclassified:
        print("\n  Each is either a divergence worth a line in one of the tables\n"
              "  above, or a drift worth fixing. Leaving it here means neither.")

    print(f"\n{'=' * 100}")
    if bad:
        print(f"FAIL: {bad} classified parameter(s) are not as they should be")
    else:
        print("OK: every classified parameter is as it should be")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
