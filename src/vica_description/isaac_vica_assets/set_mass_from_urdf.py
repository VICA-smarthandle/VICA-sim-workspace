#!/usr/bin/env python3
"""Copy link masses from the URDF into the robot USD, without re-importing.

    python3 set_mass_from_urdf.py [--check]

Re-importing is the documented way to get a URDF change into the stage, and
for a geometry change it is the only way. For a mass change it is the wrong
trade: the importer produces a fresh asset, and everything done to that asset
afterwards -- sensors attached, caster drives removed, wheel drives set, six
ROS graphs built -- is thrown away and has to be redone through a GUI step
that no script can check. The chain is four stages long and one of them is a
person clicking.

Mass lives in the USD as UsdPhysics.MassAPI attributes on the rigid bodies,
which is exactly the shape of edit fixup_vica_usd_joints.py already makes to
the joint drives. So this does the same: open, write the one thing that
changed, save. The stages reference this asset rather than copying it, so
they pick the new values up the next time they are opened.

The masses are read from urdf/vica.urdf rather than written here, so there is
one source and this file cannot become a second place the numbers live. The
URDF itself is generated from VICA.xacro by export_isaac_urdf.sh, which
stamps it -- run that first, or this reads a stale file.

Inertia is scaled rather than copied. The importer diagonalised it: the URDF
carries ixx/iyy/izz plus a non-zero ixz, and the USD carries the principal
values, which is why the two do not match term for term. Scaling the USD's own
diagonal by the mass ratio is correct for unchanged geometry and does not
require redoing that diagonalisation.
"""

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROBOT_USD = os.path.join(HERE, "robot", "vica", "vica.usda")
URDF = os.path.join(PKG, "urdf", "vica.urdf")
TOL = 1e-4


def urdf_masses(path):
    """link name -> mass, from the expanded URDF."""
    return {k: v[0] for k, v in urdf_inertials(path).items()}


def urdf_inertials(path):
    """link name -> (mass, (ixx, iyy, izz)), from the expanded URDF.

    Inertia is only needed for a link the USD has no mass for at all, where
    there is no existing diagonal to scale. Off-diagonal terms are dropped:
    a link in that state is a small sensor body whose products of inertia are
    zero in the URDF anyway, and this is not the place to diagonalise.
    """
    text = open(path).read()
    out = {}
    for m in re.finditer(
            r'<link name="([^"]+)">\s*(?:<!--.*?-->\s*)*<inertial>(.*?)</inertial>',
            text, re.S):
        name, body = m.group(1), m.group(2)
        mm = re.search(r'<mass value="([0-9.eE+-]+)"', body)
        if not mm:
            continue
        ine = re.search(r'<inertia\s+([^/]*)/>', body)
        d = {}
        if ine:
            for k in ("ixx", "iyy", "izz"):
                km = re.search(k + r'="([0-9.eE+-]+)"', ine.group(1))
                if km:
                    d[k] = float(km.group(1))
        out[name] = (float(mm.group(1)),
                     (d.get("ixx", 0.0), d.get("iyy", 0.0), d.get("izz", 0.0)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="report differences and write nothing")
    ap.add_argument("--usd", default=ROBOT_USD)
    ap.add_argument("--urdf", default=URDF)
    args = ap.parse_args()

    from pxr import Gf, Usd, UsdPhysics

    inertials = urdf_inertials(args.urdf)
    want = {k: v[0] for k, v in inertials.items()}
    if not want:
        print(f"  {args.urdf} 에서 질량을 읽지 못했습니다", file=sys.stderr)
        return 1

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"  {args.usd} 를 열 수 없습니다", file=sys.stderr)
        return 1
    stage.Load()

    changes, missing, total_before, total_after = [], [], 0.0, 0.0
    additions = []
    seen = set()
    # A rigid body with no MassAPI at all is the case this loop used to walk
    # straight past, because it only looks at prims that already have one.
    # camera_link was in that state for a month: PhysX logged "possibly invalid
    # inertia tensor of {1.0, 1.0, 1.0} and a negative mass" on every play, and
    # --check reported "차이 없음" while the two totals differed by its 0.113 kg.
    # Negative mass in an articulation is not confined to the link that has it;
    # the whole mass matrix goes, and the robot settled 6 mm low and 2.4 degrees
    # nose-up in places where it should have been level.
    for prim in stage.Traverse():
        if (prim.HasAPI(UsdPhysics.RigidBodyAPI)
                and not prim.HasAPI(UsdPhysics.MassAPI)
                and prim.GetName() in want):
            additions.append(prim)
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
        seen.add(prim.GetName())
        api = UsdPhysics.MassAPI(prim)
        have = api.GetMassAttr().Get()
        if have is None:
            continue
        name = prim.GetName()
        total_before += have
        if name not in want:
            missing.append(name)
            total_after += have
            continue
        target = want[name]
        total_after += target
        if abs(have - target) <= TOL:
            continue
        changes.append((prim, api, name, have, target))

    total_after += sum(want[p.GetName()] for p in additions)

    print(f"  URDF {os.path.basename(args.urdf)}   합계 {sum(want.values()):.4f} kg")
    print(f"  USD  {os.path.basename(args.usd)}   합계 {total_before:.4f} kg -> {total_after:.4f} kg")
    if missing:
        print(f"  [경고] URDF 에 없는 USD 링크: {missing}")

    for prim in additions:
        print(f"    {prim.GetName():28s}   질량 속성 없음 -> "
              f"{want[prim.GetName()]:.4f} kg   (새로 만듦)")

    if not changes and not additions:
        print("  차이 없음.")
        return 0

    for _, _, name, have, target in changes:
        print(f"    {name:28s} {have:9.4f} -> {target:9.4f} kg   (x{target / have:.6f})")

    if args.check:
        print("  --check 이므로 쓰지 않았습니다.")
        return 1

    for prim in additions:
        name = prim.GetName()
        mass, (ixx, iyy, izz) = inertials[name]
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(mass)
        if ixx or iyy or izz:
            api.CreateDiagonalInertiaAttr(Gf.Vec3f(ixx, iyy, izz))

    for prim, api, name, have, target in changes:
        ratio = target / have
        api.GetMassAttr().Set(target)
        di = api.GetDiagonalInertiaAttr().Get()
        if di is not None:
            api.GetDiagonalInertiaAttr().Set(type(di)(*[v * ratio for v in di]))
    stage.GetRootLayer().Save()
    print(f"  saved {args.usd}")
    print("  스테이지는 이 에셋을 참조하므로 다음에 열 때 반영됩니다.")
    print("  verify_stage 로 각 스테이지를 다시 확인하십시오.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
