#!/usr/bin/env python3
"""Find the arm pose the robot should drive with, from the constraints.

    stow_pose.py urdf/vica_arm.urdf [--prefix gen3_joint_] [--json OUT]

[TEST ONLY] Reads a URDF and prints numbers. Publishes nothing, commands
nothing, and never opens a simulator.

Why this has to be derived
--------------------------
kortex_description defines no stowed pose. config/initial_positions.yaml is
zeros for every joint, and zero on a Gen3 lite is straight up: the end effector
sits 1.382 m above base_link, which is the highest the arm's mass can be put.
Every tipping figure measured in that pose is the worst case rather than the
driving case, and there is no SRDF or MoveIt configuration in the package to
take a "home" or "retract" from.

So the pose comes from what the robot needs instead of from a catalogue.

    hard   every arm link stays inside the chassis footprint in plan view
    hard   and above the deck it stands on, so it is not inside the body
    hard   joint limits
    soft   whole-robot centre of mass as low as possible

The footprint constraint is the one that matters for driving. nav2 checks a
hexagon measured from the chassis, and an arm that hangs outside it is an
obstacle the planner cannot see. Keeping the arm inside means the passable
width of the robot is the same whether the arm is fitted or not, which is what
lets the driving results stand for both.

CoM height is the objective rather than a constraint because there is no
threshold to meet, only better and worse: forward tipping deceleration is
g * (x_front - x_com) / z_com, so every centimetre off z_com buys margin.

The search is a coarse grid over the joints that decide the fold, refined
locally. Six joints is too many to grid finely and the wrist barely moves the
mass, so joints 1 to 4 are searched and 5 and 6 are left at zero.
"""

import argparse
import itertools
import json
import math
import re
import sys

import numpy as np

# The chassis footprint, before nav2's padding, as authored in the nav2 config.
# Padding is nav2's own safety margin and is not something the arm may spend.
FOOTPRINT = [(0.305, 0.2275), (0.305, -0.2275), (-0.305, -0.2275),
             (-0.595, -0.035), (-0.595, 0.035), (-0.305, 0.2275)]


def parse_urdf(path):
    """Links with mass, and joints with origin, axis, limits and parent."""
    text = open(path).read()
    links = {}
    # Frame-only links are written <link name="x"/> with no closing tag. A
    # pattern that always looks for </link> runs past them and swallows the
    # next link's body, which silently moves that link's mass onto the frame:
    # the riser's 2 kg landed on camera_optical_frame and the total came out
    # 10.4 kg instead of 27.0. Match the self-closing form first.
    for m in re.finditer(r'<link name="([^"]+)"\s*(?:/>|>(.*?)</link>)',
                         text, re.S):
        name, body = m.group(1), m.group(2) or ""
        mass = re.search(r'<mass value="([-\d.eE+]+)"', body)
        if not mass:
            continue
        o = re.search(r'<inertial>.*?<origin([^/>]*)/?>', body, re.S)
        xyz = [0.0, 0.0, 0.0]
        if o:
            v = re.search(r'xyz="([-\d.eE+ ]+)"', o.group(1))
            if v:
                xyz = [float(q) for q in v.split()] if isinstance(v, list) else \
                    [float(q) for q in v.group(1).split()]
        links[name] = (float(mass.group(1)), np.array(xyz))

    joints = {}
    for m in re.finditer(r'<joint name="([^"]+)" type="([^"]+)">(.*?)</joint>',
                         text, re.S):
        name, kind, body = m.group(1), m.group(2), m.group(3)
        parent = re.search(r'<parent link="([^"]+)"', body).group(1)
        child = re.search(r'<child link="([^"]+)"', body).group(1)
        o = re.search(r'<origin([^/>]*)/?>', body)
        xyz, rpy = [0.0] * 3, [0.0] * 3
        if o:
            vx = re.search(r'xyz="([-\d.eE+ ]+)"', o.group(1))
            vr = re.search(r'rpy="([-\d.eE+ ]+)"', o.group(1))
            if vx:
                xyz = [float(q) for q in vx.group(1).split()]
            if vr:
                rpy = [float(q) for q in vr.group(1).split()]
        a = re.search(r'<axis xyz="([-\d.eE+ ]+)"', body)
        axis = [float(q) for q in a.group(1).split()] if a else [0.0, 0.0, 1.0]
        lim = re.search(r'<limit[^/>]*lower="([-\d.eE+]+)"[^/>]*upper="([-\d.eE+]+)"',
                        body)
        joints[name] = dict(
            kind=kind, parent=parent, child=child,
            xyz=np.array(xyz), rpy=np.array(rpy), axis=np.array(axis),
            lower=float(lim.group(1)) if lim else None,
            upper=float(lim.group(2)) if lim else None)
    return links, joints


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_matrix(axis, angle):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def forward(links, joints, angles, root="base_link"):
    """World pose of every link, with the robot's root at the origin."""
    children = {}
    for n, j in joints.items():
        children.setdefault(j["parent"], []).append(n)
    poses = {root: (np.zeros(3), np.eye(3))}
    stack = [root]
    while stack:
        parent = stack.pop()
        p_t, p_R = poses[parent]
        for jn in children.get(parent, []):
            j = joints[jn]
            R = p_R @ rpy_matrix(j["rpy"])
            t = p_t + p_R @ j["xyz"]
            if j["kind"] in ("revolute", "continuous"):
                R = R @ axis_matrix(j["axis"], angles.get(jn, 0.0))
            poses[j["child"]] = (t, R)
            stack.append(j["child"])
    return poses


# The deck the arm bolts to, measured off meshes/base_link.stl rather than
# taken from the collision box. The box says 0.259 and the mesh has no upward
# facing surface within 20 mm of that; the real top plate is 0.170, and the
# riser mounted at 0.259 stood 89 mm in the air.
DECK_Z = 0.170

# The mast is not part of the deck and the arm has to miss it. Taken from
# chassis_mast_size and chassis_mast_origin in VICA.xacro: a keep-out box the
# stowed pose may not enter. Without it "above the deck" is satisfied by poses
# that fold the arm straight through the mast the handle is bolted to.
MAST = ((-0.505, -0.265), (-0.020, 0.020), (0.259, 0.673))


def in_mast(p, margin=0.02):
    (x0, x1), (y0, y1), (z0, z1) = MAST
    return (x0 - margin < p[0] < x1 + margin
            and y0 - margin < p[1] < y1 + margin
            and z0 - margin < p[2] < z1 + margin)


def evaluate(links, joints, poses, arm_links):
    """Whole-robot CoM, how far the arm pokes out, and how far it sinks in.

    The second number is the plan-view escape. The third is the one that
    caught the first answer this script produced: folding the shoulder to
    -149.5 degrees put the gripper at z 0.186, seven centimetres under the
    deck and well inside the footprint, which is not a stowed arm but an arm
    inside the chassis. Staying in the footprint is necessary and not
    sufficient; it also has to stay on top of it.

    Both checks are on the corners of each link's mesh bounding box, not on
    link origins. Origins were tried first and were not enough: the pose they
    accepted put gen3_upper_wrist_link's origin 12 mm above the deck and its
    mesh 28 mm below it. A bounding box is larger than the mesh inside it, so
    this errs towards rejecting poses that would have fitted, which is the
    right way to be wrong here.
    """
    total = 0.0
    com = np.zeros(3)
    for name, (mass, local) in links.items():
        if name not in poses:
            continue
        t, R = poses[name]
        com += mass * (t + R @ local)
        total += mass
    com /= total

    worst = 0.0
    sunk = 0.0
    for name in arm_links:
        if name not in poses:
            continue
        t, R = poses[name]
        pts = HULLS.get(name)
        pts = (R @ pts.T).T + t if pts is not None else np.array([t])
        for p in pts:
            out = outside_by(p[0], p[1])
            worst = max(worst, out)
            if out == 0.0 and p[2] < DECK_Z:
                sunk = max(sunk, DECK_Z - p[2])
            if in_mast(p):
                sunk = max(sunk, 0.001)
    return com, total, worst, sunk


# link -> corners of its mesh bounding box, in the link frame. Filled by
# load_hulls(); empty means the search falls back to link origins and says so.
HULLS = {}


def load_hulls(urdf_path, share):
    """Bounding box corners per link, from the collision or visual mesh."""
    import os
    import struct
    text = open(urdf_path).read()
    for m in re.finditer(r'<link name="([^"]+)"\s*(?:/>|>(.*?)</link>)', text, re.S):
        name, body = m.group(1), m.group(2) or ""
        blk = re.search(r"<collision>(.*?)</collision>", body, re.S) or \
            re.search(r"<visual>(.*?)</visual>", body, re.S)
        if not blk:
            continue
        blk = blk.group(1)
        mesh = re.search(r'filename="package://([^"]+)"', blk)
        if not mesh:
            continue
        pkg = mesh.group(1).split("/")[0]
        path = os.path.join(share.get(pkg, ""), mesh.group(1)[len(pkg) + 1:])
        if not os.path.exists(path):
            continue
        with open(path, "rb") as fh:
            head = fh.read(84)
            n = struct.unpack("<I", head[80:84])[0]
            buf = fh.read(n * 50)
        if len(buf) < n * 50:          # ascii STL
            txt = open(path, errors="replace").read()
            v = re.findall(r"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", txt)
            pts = np.array(v, dtype=float)
        else:
            pts = np.array([struct.unpack("<9f", buf[i * 50 + 12:i * 50 + 48])
                            for i in range(0, n, 11)], dtype=float).reshape(-1, 3)
        sc = re.search(r'scale="([-\d.eE+ ]+)"', blk)
        if sc:
            pts = pts * np.array([float(v) for v in sc.group(1).split()])
        o = re.search(r"<origin([^/>]*)/?>", blk)
        if o:
            vr = re.search(r'rpy="([-\d.eE+ ]+)"', o.group(1))
            vx = re.search(r'xyz="([-\d.eE+ ]+)"', o.group(1))
            if vr:
                pts = (rpy_matrix(np.array([float(v) for v in vr.group(1).split()]))
                       @ pts.T).T
            if vx:
                pts = pts + np.array([float(v) for v in vx.group(1).split()])
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        HULLS[name] = np.array([[x, y, z] for x in (lo[0], hi[0])
                                for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return HULLS


def outside_by(x, y):
    """Metres beyond the footprint edge, 0 when inside."""
    poly = FOOTPRINT
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    best = min(
        seg_distance(x, y, poly[i], poly[(i + 1) % n]) for i in range(n))
    return 0.0 if inside else best


def seg_distance(x, y, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == dy == 0:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--prefix", default="gen3_joint_")
    ap.add_argument("--steps", type=int, default=9,
                    help="grid points per searched joint")
    ap.add_argument("--json", default=None)
    args = ap.parse_args(argv)

    links, joints = parse_urdf(args.urdf)
    load_hulls(args.urdf, {
        "vica_description":
            "/home/sim/vica_ws/install/vica_description/share/vica_description",
        "kortex_description": "/opt/ros/jazzy/share/kortex_description"})
    arm_joints = sorted(n for n in joints if n.startswith(args.prefix))
    if not arm_joints:
        print(f"{args.prefix} 로 시작하는 조인트가 없습니다. 팔 없는 URDF 입니까?",
              file=sys.stderr)
        return 2
    arm_links = [j["child"] for n, j in joints.items()
                 if n.startswith(args.prefix.rstrip("joint_").rstrip("_"))
                 or j["child"].startswith(args.prefix.split("joint")[0])]
    arm_links = sorted(set(arm_links))

    searched = arm_joints[:4]
    fixed = arm_joints[4:]
    print(f"  조인트 {len(arm_joints)}개 중 {len(searched)}개 탐색, "
          f"{len(fixed)}개 0 고정 (손목은 질량을 거의 안 옮깁니다)")

    grids = []
    for n in searched:
        lo, hi = joints[n]["lower"], joints[n]["upper"]
        grids.append(np.linspace(lo, hi, args.steps))

    zero = forward(links, joints, {})
    com0, total, out0, sunk0 = evaluate(links, joints, zero, arm_links)
    print(f"\n  0도 자세 : CoM z {com0[2] + 0.19:.4f} m (지면 기준), "
          f"발자국 밖 {out0 * 1000:.0f} mm, 차체 침투 {sunk0 * 1000:.0f} mm")

    best = None
    for combo in itertools.product(*grids):
        angles = dict(zip(searched, combo))
        poses = forward(links, joints, angles)
        com, _, out, sunk = evaluate(links, joints, poses, arm_links)
        # Outside the footprint and inside the chassis are both disqualifying,
        # not merely expensive.
        score = (out > 1e-6 or sunk > 1e-6, com[2])
        if best is None or score < best[0]:
            best = (score, angles, com, out, sunk)

    score, angles, com, out, sunk = best
    z_ground = com[2] + 0.19
    print(f"\n  최적 자세 : CoM z {z_ground:.4f} m, 발자국 밖 {out * 1000:.0f} mm")
    print(f"  낮아진 양 : {(com0[2] - com[2]) * 1000:.0f} mm")
    print()
    for n in arm_joints:
        v = angles.get(n, 0.0)
        lo, hi = joints[n]["lower"], joints[n]["upper"]
        print(f"    {n:<16} {v:+.4f} rad  ({math.degrees(v):+7.1f} 도)"
              f"   한계 [{math.degrees(lo):+.0f}, {math.degrees(hi):+.0f}]")

    x_front = 0.154
    print(f"\n  전방 전복 가속도 : "
          f"{9.81 * (x_front - com[0]) / z_ground:.2f} m/s²  "
          f"(0도 자세 {9.81 * (x_front - com0[0]) / (com0[2] + 0.19):.2f})")

    if args.json:
        json.dump({n: angles.get(n, 0.0) for n in arm_joints},
                  open(args.json, "w"), indent=1)
        print(f"\n  wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
