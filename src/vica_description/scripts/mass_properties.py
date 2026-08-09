#!/usr/bin/env python3
"""Whole-robot mass, centre of mass, and the deceleration that tips it over.

    mass_properties.py urdf/vica.urdf
    mass_properties.py urdf/vica.urdf urdf/vica_arm.urdf     compare two

Every number this prints used to live in a comment in VICA.xacro, hand
computed. Two of them were wrong by 15% and nothing could have caught it: a
comment cannot disagree with the file it sits in. This walks the joint tree
instead, so the tipping margin is a property of the model rather than of
whoever last edited the paragraph above it.

What it computes
----------------
CoM is accumulated through the joint chain, not read off base_link's inertial
block. base_link holds 16.6 kg of the 20 kg; the wheels and casters hold the
rest and they sit low and behind, which moves the whole-robot CoM 15 mm back
and 23 mm down from base_link's own. Both shifts matter here -- back is more
tipping margin, down is more still.

Tipping is the static limit: the deceleration at which the normal force on the
rear contacts reaches zero. Beyond it the robot rotates about the front
contact line.

    a_tip = g * (x_front - x_com) / z_com

VICA drives with +x forward, so braking throws the load onto the drive wheels
at x = +0.154 and the casters at x = -0.252 lift. That is the case that
matters: this robot never reverses, and a person is holding a handle behind
it, so a rearward tip is a different and much worse problem than a forward one
-- but it is also the one that needs a hard acceleration the controller cannot
produce.

Rotation is ignored. A differential drive turning at its 1.0 rad/s limit puts
about 0.02 g of lateral load on a 0.18 m half-track, which is two orders below
the limits printed here.
"""

import math
import sys
import xml.etree.ElementTree as ET

G = 9.81

# What the driving stack will actually ask for. The tipping limits below are
# only interesting next to the deceleration that can be commanded.
BRAKE_LIMITS = [
    ("velocity_smoother max_decel", 2.5),
    ("MPPI ax_max", 2.0),
]


def _xyz(node, attr="xyz", default=(0.0, 0.0, 0.0)):
    if node is None:
        return default
    raw = node.get(attr)
    if raw is None:
        return default
    return tuple(float(v) for v in raw.split())


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def _apply(rot, trans, v):
    return tuple(trans[i] + sum(rot[i][j] * v[j] for j in range(3))
                 for i in range(3))


def link_origins(root):
    """Every link's pose in base_link coordinates.

    Joint rpy is honoured. It is zero everywhere in this model today, but a
    riser or an arm mount is exactly the kind of thing someone tilts, and a
    CoM tool that silently ignored the tilt would be worse than no tool.
    """
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        joints.append((
            j.find("parent").get("link"),
            j.find("child").get("link"),
            _xyz(o),
            _rpy_matrix(*_xyz(o, "rpy")),
        ))

    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    poses = {"base_link": ((0.0, 0.0, 0.0), identity)}
    # Repeat until nothing new resolves: the joint list is in file order, not
    # tree order, and a child can appear before its parent.
    for _ in range(len(joints) + 1):
        for parent, child, t, r in joints:
            if parent in poses and child not in poses:
                pt, pr = poses[parent]
                poses[child] = (
                    _apply(pr, pt, t),
                    tuple(tuple(sum(pr[i][k] * r[k][j] for k in range(3))
                                for j in range(3)) for i in range(3)),
                )
    missing = {c for _, c, _, _ in joints} - set(poses)
    if missing:
        raise SystemExit(f"base_link 에서 닿지 않는 링크: {sorted(missing)}")
    return poses


def mass_properties(path):
    root = ET.parse(path).getroot()
    poses = link_origins(root)

    total = 0.0
    moment = [0.0, 0.0, 0.0]
    parts = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        m = float(inertial.find("mass").get("value"))
        if m <= 0.0:
            continue
        t, r = poses[link.get("name")]
        c = _apply(r, t, _xyz(inertial.find("origin")))
        total += m
        for i in range(3):
            moment[i] += m * c[i]
        parts.append((link.get("name"), m, c))

    com = tuple(v / total for v in moment)

    # Ground contact in x, taken from the wheel links rather than a constant.
    # A riser that moved the wheels would otherwise leave this reading the old
    # wheelbase with no sign that it had.
    xs = [poses[l.get("name")][0][0] for l in root.findall("link")
          if "wheel_1" in l.get("name")]
    return {
        "path": path, "mass": total, "com": com, "parts": parts,
        "front_x": max(xs), "rear_x": min(xs),
        # Ground height of base_link. base_footprint is base_link's *parent*,
        # so the tree walk above never reaches it -- read the joint directly
        # rather than expecting it in poses.
        "base_height": next(
            _xyz(j.find("origin"))[2] for j in root.findall("joint")
            if j.find("child").get("link") == "base_link"),
    }


def report(p):
    z = p["com"][2] + p["base_height"]
    fwd = (p["front_x"] - p["com"][0])
    rear = (p["com"][0] - p["rear_x"])
    print(f"\n=== {p['path']}")
    print(f"  총 질량        {p['mass']:.3f} kg   ({len(p['parts'])} 링크)")
    print(f"  무게중심       x {p['com'][0]:+.4f}  y {p['com'][1]:+.4f}  "
          f"z {p['com'][2]:+.4f}  (base_link 기준)")
    print(f"  무게중심 높이  {z:.4f} m  (지면 기준)")
    print(f"  접지선         앞 {p['front_x']:+.3f}  뒤 {p['rear_x']:+.3f}")
    if p["com"][0] > p["front_x"] or p["com"][0] < p["rear_x"]:
        print("  🔴 무게중심이 지지 다각형 밖입니다 -- 정지 상태에서 넘어집니다")
    a_f, a_r = G * fwd / z, G * rear / z
    print(f"  전방 전복      {a_f:6.2f} m/s²   (여유 팔 {fwd:.4f} m)")
    print(f"  후방 전복      {a_r:6.2f} m/s²   (여유 팔 {rear:.4f} m)")
    for label, limit in BRAKE_LIMITS:
        print(f"    vs {label:28s} {limit:.1f} m/s²  ->  {a_f / limit:5.2f} 배")
    heavy = sorted(p["parts"], key=lambda q: -q[1])[:4]
    print("  질량 상위:")
    for name, m, c in heavy:
        print(f"    {name:28s} {m:7.3f} kg  x {c[0]:+.3f}  z {c[2]:+.3f}")
    return p


def main(argv):
    if len(argv) < 2:
        raise SystemExit(__doc__.strip().splitlines()[2].strip())
    done = [report(mass_properties(a)) for a in argv[1:]]
    if len(done) == 2:
        a, b = done
        za = a["com"][2] + a["base_height"]
        zb = b["com"][2] + b["base_height"]
        print(f"\n=== 차이")
        print(f"  질량      {a['mass']:.2f} -> {b['mass']:.2f} kg  "
              f"({b['mass'] - a['mass']:+.2f})")
        print(f"  CoM 높이  {za:.3f} -> {zb:.3f} m  ({zb - za:+.3f})")
        print(f"  전방 전복 {G * (a['front_x'] - a['com'][0]) / za:.2f} -> "
              f"{G * (b['front_x'] - b['com'][0]) / zb:.2f} m/s²")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
