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
    text = open(path).read()
    out = {}
    for m in re.finditer(
            r'<link name="([^"]+)">\s*(?:<!--.*?-->\s*)?<inertial>.*?'
            r'<mass value="([0-9.eE+-]+)"', text, re.S):
        out[m.group(1)] = float(m.group(2))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="report differences and write nothing")
    ap.add_argument("--usd", default=ROBOT_USD)
    ap.add_argument("--urdf", default=URDF)
    args = ap.parse_args()

    from pxr import Usd, UsdPhysics

    want = urdf_masses(args.urdf)
    if not want:
        print(f"  {args.urdf} 에서 질량을 읽지 못했습니다", file=sys.stderr)
        return 1

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"  {args.usd} 를 열 수 없습니다", file=sys.stderr)
        return 1
    stage.Load()

    changes, missing, total_before, total_after = [], [], 0.0, 0.0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
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

    print(f"  URDF {os.path.basename(args.urdf)}   합계 {sum(want.values()):.4f} kg")
    print(f"  USD  {os.path.basename(args.usd)}   합계 {total_before:.4f} kg -> {total_after:.4f} kg")
    if missing:
        print(f"  [경고] URDF 에 없는 USD 링크: {missing}")

    if not changes:
        print("  차이 없음.")
        return 0

    for _, _, name, have, target in changes:
        print(f"    {name:28s} {have:9.4f} -> {target:9.4f} kg   (x{target / have:.6f})")

    if args.check:
        print("  --check 이므로 쓰지 않았습니다.")
        return 1

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
