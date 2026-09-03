#!/usr/bin/env python3
"""What the arm asks of the shoulder servo, from the expanded URDF.

    arm_torque.py urdf/vica_arm.urdf
    arm_torque.py urdf/vica_arm.urdf --payload 0.5

Why this exists
---------------
Lengthening the arm is nearly free in the model and not free at all in the
motor. Every OMX joint is the same DYNAMIXEL XM430-W350-T, and joint2 -- the
shoulder -- holds everything past it at whatever distance it is held out.
Doubling a link doubles that distance and roughly doubles the mass hanging on
it, so the torque goes up about fourfold while the servo stays where it was.

That number cannot live in a comment. A comment cannot disagree with the
lengths in the file above it, and the whole point of the lengths being
parameters is that they change.

What it computes
----------------
The static case that costs the most: the arm held straight out horizontally,
joint2 carrying every link past it plus a payload at the gripper. Gravity
only. There is no dynamic term because the servo is geared to 46 RPM and
nothing on this arm is thrown.

    tau = sum over links past joint2 of m * g * (distance from joint2)

Distance is along the arm, which is what a horizontal pose makes it. Link
centres of mass come from each link's own inertial origin, walked through the
joint chain, so a link whose mass was scaled with its length is counted with
the mass it actually has in the file.

Stall is 4.1 N.m at 12 V. Stall is not a rating: it is where the motor stops
turning and starts heating, so a pose over about half of it holds only
briefly. The URDF's own effort limit, 1.5, is ROBOTIS's working figure and is
the one a planner should be given.
"""

import argparse
import math
import xml.etree.ElementTree as ET

G = 9.81
STALL_NM = 4.1        # XM430-W350-T at 12.0 V
WORKING_NM = 1.5      # what ROBOTIS's own URDF declares as the effort limit


def parse(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        i = l.find("inertial")
        if i is None:
            links[l.get("name")] = (0.0, (0.0, 0.0, 0.0))
            continue
        m = float(i.find("mass").get("value"))
        o = i.find("origin")
        xyz = tuple(float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split())
        links[l.get("name")] = (m, xyz)
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = tuple(float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split())
        joints[j.find("child").get("link")] = (j.find("parent").get("link"),
                                               j.get("name"), xyz)
    return links, joints


def offset_from(links, joints, link, root):
    """Distance along the chain from `root` to `link`'s origin.

    Straight-line sum of the joint offsets, which is what a straight arm is.
    """
    d = 0.0
    cur = link
    while cur != root:
        if cur not in joints:
            return None
        parent, _, xyz = joints[cur]
        d += math.sqrt(sum(v * v for v in xyz))
        cur = parent
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf")
    ap.add_argument("--payload", type=float, default=0.5, metavar="KG",
                    help="mass at the gripper. 0.5 kg is the OMX catalogue "
                         "figure (default: 0.5)")
    ap.add_argument("--prefix", default="omx_")
    args = ap.parse_args()

    links, joints = parse(args.urdf)
    shoulder = None
    for child, (parent, name, _) in joints.items():
        if name == f"{args.prefix}joint2":
            shoulder = child
            break
    if shoulder is None:
        raise SystemExit(f"{args.prefix}joint2 이 {args.urdf} 에 없습니다")
    root = joints[shoulder][0]

    rows, tau = [], 0.0
    tip = 0.0
    for link in links:
        d = offset_from(links, joints, link, root)
        if d is None:
            continue
        m, com = links[link]
        # The link's own centre, along the same straight arm.
        r = d + math.sqrt(sum(v * v for v in com))
        tip = max(tip, d)
        if m <= 0:
            continue
        rows.append((link, m, r, m * G * r))
        tau += m * G * r

    rows.sort(key=lambda r: r[2])
    print(f"{args.urdf}  (joint2 를 원점으로, 팔을 수평으로 뻗은 자세)\n")
    print(f"  {'link':28s} {'질량 kg':>8s} {'거리 m':>8s} {'토크 N.m':>9s}")
    for name, m, r, t in rows:
        print(f"  {name:28s} {m:8.3f} {r:8.3f} {t:9.2f}")
    print(f"  {'':28s} {sum(r[1] for r in rows):8.3f} {'':8s} {tau:9.2f}  팔 자체")

    pay = args.payload * G * tip
    print(f"  {'payload':28s} {args.payload:8.3f} {tip:8.3f} {pay:9.2f}")
    print(f"\n  합계 {tau + pay:.2f} N.m")
    print(f"  XM430-W350-T 스톨 {STALL_NM} N.m 대비 "
          f"{(tau + pay) / STALL_NM * 100:.0f}%"
          f"   (팔만 {tau / STALL_NM * 100:.0f}%)")
    print(f"  URDF 상시 한계 {WORKING_NM} N.m 대비 "
          f"{(tau + pay) / WORKING_NM * 100:.0f}%")
    if tau + pay > STALL_NM:
        print("\n  이 자세는 나오지 않습니다. 모터가 버티는 값을 넘었습니다.")
    elif tau + pay > STALL_NM / 2:
        print("\n  잠깐은 되고 계속 들고 있지는 못합니다. 스톨의 절반을 넘었습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
