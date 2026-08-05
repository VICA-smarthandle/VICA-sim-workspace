"""Report what physics actually sees in the currently open stage.

Paste into Isaac's Script Editor (Window > Script Editor) with the stage open,
and run it before pressing Play. It prints the same facts a headless check
prints, so a stage that behaves one way here and another way there can be
compared line by line instead of by description.

It changes nothing.

The reason this exists: the robot rests correctly headless and falls through
the floor in the GUI. Everything below is a thing that would explain that, and
each is cheap to read and easy to get wrong by eye in the outliner.
"""

from pxr import Usd, UsdGeom, UsdPhysics

import omni.usd


ARTICULATION_SUFFIX = "/Geometry/base_footprint/base_link"


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("No stage open.")
        return

    print("=" * 68)
    print("root layer :", stage.GetRootLayer().identifier)
    print("up axis    :", UsdGeom.GetStageUpAxis(stage))
    print("metersPerUnit:", UsdGeom.GetStageMetersPerUnit(stage))

    # ---- physics scenes ---------------------------------------------------
    # More than one is a problem: bodies and ground can end up in different
    # scenes, which look identical in the outliner and never collide.
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    print(f"\nphysics scenes ({len(scenes)}):")
    for s in scenes:
        d = s.GetAttribute("physics:gravityDirection")
        m = s.GetAttribute("physics:gravityMagnitude")
        print(f"    {s.GetPath()}")
        print(f"        gravityDirection={d.Get() if d else 'unset'} "
              f"gravityMagnitude={m.Get() if m else 'unset'}")
    print("    note: (0,0,0) and -inf are USD's 'use the default' sentinels,")
    print("          not an absence of gravity.")
    if len(scenes) != 1:
        print("    WARNING: expected exactly one physics scene")

    # ---- the robot as physics sees it -------------------------------------
    rigid = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    arts = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]

    print(f"\nRigidBodyAPI      : {len(rigid)}   (expect 9)")
    print(f"RevoluteJoint     : {len(joints)}   (expect 6)")
    print(f"ArticulationRoot  : {len(arts)}   (expect 1)")
    print(f"CollisionAPI      : {len(colliders)}   (expect 134 with the hospital)")
    for a in arts:
        print(f"    articulation root: {a.GetPath()}")
    if not rigid:
        print("    >>> No rigid bodies. Nothing here is simulated, so no ground")
        print("        can hold the robot up and adding one will not help.")
        print("        Check the Physics variant below.")

    # ---- the variant that decides whether physics exists ------------------
    robot = None
    for p in stage.Traverse():
        if str(p.GetPath()).endswith(ARTICULATION_SUFFIX):
            robot = p
            break
    if robot is None:
        for p in stage.Traverse():
            if p.GetName() == "VICA":
                robot = p
                break
    print("\nPhysics variant:")
    found_vset = False
    node = robot
    while node and node.GetPath() != node.GetPath().GetParentPath():
        vsets = node.GetVariantSets()
        if "Physics" in vsets.GetNames():
            sel = vsets.GetVariantSet("Physics").GetVariantSelection()
            print(f"    {node.GetPath()}  Physics = '{sel}'")
            if sel != "physx":
                print("    >>> should be 'physx'. 'none' removes every rigid body")
                print("        and joint; the colliders stay visible regardless,")
                print("        because those come in through a reference.")
            found_vset = True
            break
        node = node.GetParent()
    if not found_vset:
        print("    no Physics variant set found on the robot or its ancestors")

    # ---- payload state ----------------------------------------------------
    # The variant's content arrives as a payload. Unloaded, it takes the rigid
    # bodies with it.
    print("\nunloaded payloads:")
    unloaded = [str(p) for p in stage.FindLoadable() if not stage.GetPrimAtPath(p).IsLoaded()]
    if unloaded:
        for u in unloaded[:12]:
            print(f"    {u}")
        print("    >>> load these (right-click > Load in the outliner) before Play")
    else:
        print("    none -- everything loadable is loaded")

    # ---- ground -----------------------------------------------------------
    planes = [
        p for p in stage.Traverse()
        if p.GetTypeName() == "Plane" and p.HasAPI(UsdPhysics.CollisionAPI)
    ]
    print(f"\ncollision planes ({len(planes)}):")
    for p in planes:
        print(f"    {p.GetPath()}  active={p.IsActive()}")

    # ---- where the robot is right now -------------------------------------
    if robot is not None:
        m = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        print(f"\nbase_link world position: "
              f"({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f})")
        print("    0.19 is correct when resting: wheel centre 0.065 = wheel radius.")
    print("=" * 68)


main()
