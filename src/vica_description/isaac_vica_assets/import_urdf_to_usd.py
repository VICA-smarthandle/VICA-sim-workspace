"""Turn a URDF into the robot USD the stages reference, without the GUI.

    $ISAAC_SIM/python.sh import_urdf_to_usd.py [--arm] [--out DIR]

      --arm      import urdf/vica_arm.urdf into robot/vica_arm/
                 (default: urdf/vica.urdf into robot/vica/)
      --out DIR  override the destination directory
      --dry-run  print what would be imported and stop

Step 2 of the chain in export_isaac_urdf.sh was written as "Isaac URDF
Importer", meaning a person clicking through a window. That was tolerable while
there was one robot. There are two now, they are rebuilt whenever VICA.xacro
changes, and the step between them was the only one nobody could repeat or
review. A GUI import also leaves no record of which options were ticked, and
those options decide whether the robot can move at all.

The three that matter here, and why:

    fix_base = False        A mobile base must be free. Left at the importer's
                            default the robot gets a fixed joint to the world
                            and sits there while the wheels turn.

    merge_fixed_joints      Off. Merging would fold laser_frame and the four
                            camera frames into base_link, and those frame names
                            are what the ROS graphs and nvblox subscribe by.

    collision_from_visuals  Off. The URDF already carries explicit collision
                            geometry, and it differs from the visual on purpose
                            -- the casters collide as 0.042 m cylinders, the
                            ground contact radius, not as their meshes.

package:// is resolved from ros_package_paths rather than from the environment,
so an import does not depend on which workspace happened to be sourced. The arm
variant needs kortex_description as well as vica_description.

What this does not do: joint drives, sensors and ROS graphs. Those are steps 3
and 4 and they already have scripts. This writes the same asset the GUI wrote,
so those scripts do not change.
"""

import argparse
import os
import shutil
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.join(PKG, "isaac_vica_assets")


def parse_args(argv):
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--arm", action="store_true",
                    help="import the arm variant instead of the driving robot")
    ap.add_argument("--out", default=None, help="destination directory")
    ap.add_argument("--dry-run", action="store_true")
    return ap.parse_args(argv)


def resolve(args):
    """The destination is the parent directory, not the asset's own folder.

    The importer creates a folder named after the URDF underneath usd_path and
    writes into that, so passing robot/vica_arm produces
    robot/vica_arm/vica_arm/vica_arm.usda. The existing asset that the stages
    reference is robot/vica/vica.usda, so the parent is what matches it.
    """
    variant = "arm" if args.arm else "base"
    urdf = os.path.join(PKG, "urdf", "vica_arm.urdf" if args.arm else "vica.urdf")
    out = args.out or os.path.join(HERE, "robot")
    return variant, urdf, out


def stamp_of(urdf_path):
    """The source hash export_isaac_urdf.sh wrote, so the USD can name it too."""
    with open(urdf_path) as fh:
        for _ in range(4):
            line = fh.readline()
            if "vica-source-hash" in line:
                return line.split("vica-source-hash")[1].strip(" ->\n")
    return None


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    variant, urdf, out = resolve(args)

    if not os.path.isfile(urdf):
        print(f"URDF 가 없습니다: {urdf}\n"
              f"  먼저 export_isaac_urdf.sh {'--arm' if args.arm else ''} 를 실행하십시오.",
              file=sys.stderr)
        return 2

    # The importer never overwrites. Given robot/ with vica_arm already in it
    # it writes vica_arm_1 beside it and says so only in a line nobody reads,
    # and the stage builders go on referencing the old asset -- so an arm that
    # was just relengthened renders at its old length and nothing looks wrong.
    # Move the old one aside instead, keeping exactly one generation, because
    # deleting it outright is how an asset got destroyed here when the export
    # that was supposed to replace it had already failed.
    asset = os.path.join(out, "vica_arm" if args.arm else "vica")
    if os.path.isdir(asset) and not args.dry_run:
        prev = asset + ".prev"
        if os.path.isdir(prev):
            shutil.rmtree(prev)
        os.rename(asset, prev)
        print(f"  이전 자산 {os.path.relpath(prev, PKG)} 로 옮겼습니다")

    hash_ = stamp_of(urdf)
    print(f"  변형     {variant}")
    print(f"  입력     {os.path.relpath(urdf, PKG)}")
    print(f"  출력     {os.path.relpath(out, PKG)}")
    print(f"  소스해시 {hash_ or '없음 (export_isaac_urdf.sh 로 만든 파일이 아닙니다)'}")
    if args.dry_run:
        return 0

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    try:
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        packages = [{"name": "vica_description", "path": PKG}]
        # The arm's meshes live in kortex_description, which is an installed
        # ROS package rather than part of this repository. Resolving it here
        # keeps the import independent of the caller's environment.
        if args.arm:
            kortex = "/opt/ros/jazzy/share/kortex_description"
            if not os.path.isdir(kortex):
                print(f"kortex_description 이 없습니다: {kortex}\n"
                      f"  ros-jazzy-kortex-description 를 설치해야 팔 메시가 들어갑니다.",
                      file=sys.stderr)
                return 3
            packages.append({"name": "kortex_description", "path": kortex})

        os.makedirs(out, exist_ok=True)
        config = URDFImporterConfig(
            urdf_path=urdf,
            usd_path=out,
            fix_base=False,
            merge_fixed_joints=False,
            merge_mesh=False,
            collision_from_visuals=False,
            allow_self_collision=False,
            ros_package_paths=packages,
        )
        written = URDFImporter(config).import_urdf()
        print(f"\n  wrote {written}")

        # Say what came out, because an import that silently drops the
        # articulation or a joint looks exactly like one that worked.
        from pxr import Usd, UsdPhysics
        stage = Usd.Stage.Open(written)
        stage.Load()
        roots, revolute, cont = [], [], []
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                roots.append(str(prim.GetPath()))
            if prim.IsA(UsdPhysics.RevoluteJoint):
                (cont if prim.GetName().endswith("wheel_joint") else revolute).append(
                    prim.GetName())
        print(f"    articulation root {len(roots)}: {roots}")
        print(f"    회전 조인트 {len(revolute) + len(cont)} 개")
        for n in sorted(revolute)[:8]:
            print(f"      {n}")
        if not roots:
            print("  ✗ articulation root 가 없습니다. 이 자산은 움직이지 않습니다.",
                  file=sys.stderr)
            return 4
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
