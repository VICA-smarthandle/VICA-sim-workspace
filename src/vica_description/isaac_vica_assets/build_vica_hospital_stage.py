"""Compose the hospital stage: downloaded environment + freshly imported VICA.

Run with Isaac's Python, no GUI needed:

    /home/sim/isaacsim/python.sh build_vica_hospital_stage.py

Why this exists rather than a saved stage
-----------------------------------------
VICA was previously put into the hospital by editing
Downloads/Environments/Hospital/hospital.usd in place. That has three costs:
re-downloading the environment silently discards the robot, there is no file
that means "VICA in the hospital", and the robot reference inside it is pinned
to whichever import happened to be current at the time -- which is how the
stage ended up still loading a robot with a 0.293 m wheel separation long after
the URDF said 0.364 m.

So this writes a new stage that references both and owns neither. hospital.usd
is never opened for writing.

Structure
---------
    /World                      default prim
      Hospital                  -> hospital.usd
        vica2                   deactivated: the stale robot baked into the
                                environment, overridden off from here so the
                                environment file itself stays untouched
      VICA                      -> robot/vica/vica.usda
      PhysicsScene              hospital.usd carries none of its own; the only
                                one in it today arrives via that stale robot
      Environment/defaultLight

Order: export_isaac_urdf.sh -> URDF importer -> fixup_vica_usd_joints.py ->
this -> build_vica_ros_graphs.py.
"""

import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

# --------------------------------------------------------------------------
# Paths
# --------------------------------------------------------------------------
try:
    # Isaac's Script Editor executes source text rather than a module, so
    # __file__ does not exist there. Same under exec() in a headless runner.
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = "/home/sim/vica_ws/src/vica_description/isaac_vica_assets"

ENVIRONMENT = "/home/sim/Downloads/Environments/Hospital/hospital.usd"
ROBOT = os.path.join(HERE, "robot", "vica", "vica.usda")
OUTPUT = os.path.join(HERE, "vica_hospital.usd")

# The prim inside the environment that holds the stale robot. Deactivated
# rather than removed: it is composed from a reference, so it cannot be removed
# from here, and overriding it off is the operation that actually works.
STALE_ROBOT_PRIM = "vica2"

# VICA spawns at the world origin, which is where the saved hospital map places
# it and where amcl's initial pose expects it. maps/hospital.yaml records the
# check that (0,0) is free space there.
ROBOT_TRANSLATE = Gf.Vec3d(0.0, 0.0, 0.0)

# The variant that carries the robot's rigid bodies and joints. See the note
# where it is applied.
PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"


def _relative(target, start_dir):
    """Asset paths relative to the stage, so the tree can be moved as a unit."""
    return os.path.relpath(target, start_dir)


def main():
    for path, label in ((ENVIRONMENT, "environment"), (ROBOT, "robot")):
        if not os.path.exists(path):
            raise RuntimeError(f"{label} not found: {path}")

    if os.path.exists(OUTPUT):
        os.remove(OUTPUT)

    stage = Usd.Stage.CreateNew(OUTPUT)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # ---- environment ------------------------------------------------------
    hospital = UsdGeom.Xform.Define(stage, "/World/Hospital")
    hospital.GetPrim().GetReferences().AddReference(
        _relative(ENVIRONMENT, HERE)
    )

    stale = stage.OverridePrim(f"/World/Hospital/{STALE_ROBOT_PRIM}")
    if stale:
        stale.SetActive(False)

    # ---- robot ------------------------------------------------------------
    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetReferences().AddReference(_relative(ROBOT, HERE))
    robot.AddTranslateOp().Set(ROBOT_TRANSLATE)

    # Pin the Physics variant here rather than inheriting it from the reference.
    #
    # The importer puts every rigid body and joint behind a variant set with
    # four options, one of which is "none". Selecting "none" -- or failing to
    # select anything -- leaves the robot with colliders and no rigid bodies at
    # all, so it is not a physics object and drops through the floor. The
    # selection is a dropdown in the GUI's property panel, one click from
    # silently disabling physics, and the failure looks like a broken ground
    # plane rather than a changed variant.
    #
    # "physx" is the right one: physx.usda sublayers physics.usda, so it is the
    # generic physics plus the PhysX-specific settings.
    vset = robot.GetPrim().GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
    if vset.IsValid():
        vset.SetVariantSelection(PHYSICS_VARIANT)
        print(f"physics variant     : {PHYSICS_VARIANT_SET} = {vset.GetVariantSelection()}")
    else:
        print(f"physics variant     : WARNING no '{PHYSICS_VARIANT_SET}' variant set on the robot")

    # ---- physics and light ------------------------------------------------
    # Gravity is authored, not left to the defaults.
    #
    # UsdPhysics.Scene.Define alone leaves gravityDirection (0,0,0) and
    # gravityMagnitude -inf. Those read as "use the stage default" only to code
    # that resolves them -- isaacsim.core's SimulationContext does, which is why
    # a headless run drops and lands correctly. Press Play in the GUI and the
    # authored values are taken at face value: direction (0,0,0) scaled by -inf
    # is not a fallback, it is NaN, and a body under NaN gravity leaves the
    # floor behind on the first step no matter what colliders are underneath it.
    #
    # That is the difference between a stage that behaves headless and falls
    # through the world in the GUI while every prim in it inspects as correct.
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Only light the scene if the environment does not. hospital.usd carries a
    # DomeLight of its own, and adding a second one on top of it just doubles
    # the exposure -- worth avoiding while chasing a viewport that flashes white.
    has_light = any(
        p.IsA(UsdLux.DomeLight) or p.IsA(UsdLux.DistantLight)
        for p in stage.Traverse()
        if p.IsActive()
    )
    if has_light:
        print("environment light   : present, none added")
    else:
        light = UsdLux.DistantLight.Define(stage, "/World/Environment/defaultLight")
        light.CreateIntensityAttr(1000.0)
        print("environment light   : none found, DistantLight added")

    stage.GetRootLayer().Save()

    # ---- report -----------------------------------------------------------
    check = Usd.Stage.Open(OUTPUT, load=Usd.Stage.LoadAll)
    arts = [
        str(p.GetPath())
        for p in check.Traverse()
        if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    colliders = sum(
        1 for p in check.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
    )
    print(f"wrote               : {OUTPUT}")
    print(f"environment         : {_relative(ENVIRONMENT, HERE)}")
    print(f"robot               : {_relative(ROBOT, HERE)}")
    print(f"stale robot         : /World/Hospital/{STALE_ROBOT_PRIM} deactivated")
    print(f"articulation roots  : {arts}")
    print(f"colliders in stage  : {colliders}")
    if len(arts) != 1:
        print(
            "WARNING: expected exactly one articulation root. More than one "
            "means the stale robot is still active; none means the robot "
            "reference did not resolve."
        )
    print("\nNext: build_vica_ros_graphs.py against this stage.")


main()
