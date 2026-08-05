"""Restore VICA's joint drive setup after a URDF re-import.

Run from Isaac Sim's Script Editor with the Timeline stopped, once per import.

The URDF importer gives every revolute joint an angular drive. That is wrong for
the four caster joints: a caster is passive, it swivels and rolls because the
robot pushes it, and a drive fights that. Left in place the casters resist
turning, the robot ploughs instead of steering, and the two drive wheels end up
working against four powered joints.

This was fixed by hand once, directly in the USD. That fix does not survive the
next import, and neither does any record of it -- the same failure mode as the
Action Graphs that lived only in the stage. Hence this file: geometry lives in
VICA.xacro, the ROS interface in build_vica_ros_graphs.py, and the joint drives
here, so a re-import is three scripted steps rather than an archaeology exercise.

What it does:

    caster joints  drive removed, and its attributes zeroed as a fallback for
                   when the joint is composed from a reference and the schema
                   cannot actually be dropped from the root layer
    drive wheels   velocity drive, zero stiffness, damping and max force set
                   below so they can be tuned in one place

Run build_vica_ros_graphs.py after this one.
"""

from pxr import Sdf, Usd, UsdPhysics, UsdShade

import omni.timeline
import omni.usd


# --------------------------------------------------------------------------
# Joints -- names come from urdf/VICA.xacro
# --------------------------------------------------------------------------
DRIVE_JOINTS = ["left_wheel_joint", "right_wheel_joint"]

# Passive. Named front_* in the URDF even though both casters sit behind the
# drive axle at x = -0.222; the names match the physical robot's package and are
# left alone for that reason. If they ever change, the error path below prints
# every revolute joint in the stage.
CASTER_JOINTS = [
    "front_left_caster_steer_joint",
    "front_right_caster_steer_joint",
    "front_left_caster_wheel_joint",
    "front_right_caster_wheel_joint",
]

# --------------------------------------------------------------------------
# Drive tuning -- the two values worth touching after a drive test
# --------------------------------------------------------------------------
# Velocity control wants zero stiffness: stiffness is the position gain, and a
# non-zero value makes the joint fight to hold an angle it was never told to.
WHEEL_STIFFNESS = 0.0
WHEEL_DAMPING = 1.0e4
WHEEL_MAX_FORCE = 1.0e5

# --------------------------------------------------------------------------
# Tyre friction
# --------------------------------------------------------------------------
# The importer binds no physics material at all, so every wheel runs on PhysX's
# default. That is enough to creep forwards -- rolling asks little of the
# contact -- and not enough to turn: rotating in place needs the drive wheels
# to push sideways against the ground hard enough to drag two casters around an
# arc, and without grip they simply spin. Commanded 0.5 rad/s produced 0.08
# degrees of yaw in ten seconds while both wheels turned in opposite
# directions, which is what no traction looks like from the outside.
#
# Rubber on a hard indoor floor. Restitution 0 because a bouncing wheel is not
# a thing this robot does.
WHEEL_STATIC_FRICTION = 0.9
WHEEL_DYNAMIC_FRICTION = 0.8
WHEEL_RESTITUTION = 0.0
# Placed under the stage's default prim so it lands inside the robot asset
# and travels with it into any stage that references the robot.
FRICTION_MATERIAL_NAME = "PhysicsMaterials/VicaTyre"

SAVE_STAGE = True


def _require_stopped_timeline():
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Stop the Timeline before editing joint drives.")


def _collect_joints(stage):
    """Map joint name -> prim for every revolute joint in the stage."""
    found = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            found.setdefault(prim.GetName(), prim)
    return found


def _set_attr(prim, name, value):
    attr = prim.GetAttribute(name)
    if attr:
        attr.Set(value)
        return True
    return False


def _disable_drive(prim):
    """Drop the angular drive, and zero it in case the schema cannot be dropped.

    A joint composed from a referenced layer keeps its referenced opinion when
    the root layer removes the API, so RemoveAPI alone is not enough. Attribute
    overrides do compose, which is why the zeroing is not merely belt and braces.
    """
    removed = False
    try:
        removed = prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
    except Exception:  # noqa: BLE001 -- older schema builds raise rather than return False
        removed = False

    zeroed = []
    for suffix, value in (
        ("stiffness", 0.0),
        ("damping", 0.0),
        ("maxForce", 0.0),
        ("targetVelocity", 0.0),
    ):
        if _set_attr(prim, f"drive:angular:physics:{suffix}", value):
            zeroed.append(suffix)
    return removed, zeroed


def _configure_velocity_drive(prim):
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(WHEEL_STIFFNESS)
    drive.CreateDampingAttr().Set(WHEEL_DAMPING)
    drive.CreateMaxForceAttr().Set(WHEEL_MAX_FORCE)
    drive.CreateTargetVelocityAttr().Set(0.0)


def _bind_wheel_friction(stage):
    """Give every wheel collider a tyre material.

    Bound on the robot asset rather than the stage so it travels with the
    robot: a material authored in one composed stage is one more thing to
    remember when building the next.
    """
    default_prim = stage.GetDefaultPrim()
    root = default_prim.GetPath() if default_prim else Sdf.Path("/")
    path = root.AppendPath(FRICTION_MATERIAL_NAME)
    material = UsdShade.Material.Define(stage, path)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(WHEEL_STATIC_FRICTION)
    physics_material.CreateDynamicFrictionAttr().Set(WHEEL_DYNAMIC_FRICTION)
    physics_material.CreateRestitutionAttr().Set(WHEEL_RESTITUTION)

    bound = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        # The collider is a child shape of the link, so the link name is the
        # parent's: .../left_wheel_1/cylinder.
        owner = prim.GetParent().GetName()
        if "wheel" not in owner:
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        bound.append(owner)
    return bound, str(path)


def main():
    _require_stopped_timeline()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    joints = _collect_joints(stage)

    missing = [n for n in DRIVE_JOINTS if n not in joints]
    if missing:
        raise RuntimeError(
            f"Drive joints not found: {missing}. "
            f"Revolute joints in the stage: {sorted(joints)}"
        )

    print("=== passive caster joints")
    touched = 0
    for name in CASTER_JOINTS:
        prim = joints.get(name)
        if prim is None:
            continue
        removed, zeroed = _disable_drive(prim)
        touched += 1
        state = "DriveAPI removed" if removed else "DriveAPI kept (referenced)"
        print(f"    {name:34s} {state}, zeroed: {','.join(zeroed) or 'none'}")
    if touched == 0:
        print("    none found -- check the caster joint names against the URDF")
    elif touched != 4:
        print(f"    WARNING: expected 4 caster joints, handled {touched}")

    print("\n=== drive wheels")
    for name in DRIVE_JOINTS:
        _configure_velocity_drive(joints[name])
        print(
            f"    {name:34s} velocity drive  stiffness={WHEEL_STIFFNESS} "
            f"damping={WHEEL_DAMPING} maxForce={WHEEL_MAX_FORCE}"
        )

    print("\n=== tyre friction")
    bound, material_path = _bind_wheel_friction(stage)
    if bound:
        print(f"    {material_path}  "
              f"static={WHEEL_STATIC_FRICTION} dynamic={WHEEL_DYNAMIC_FRICTION}")
        print(f"    bound to {len(bound)} wheel colliders: {', '.join(sorted(set(bound)))}")
    else:
        print("    WARNING no wheel colliders found to bind")

    if SAVE_STAGE:
        stage.GetRootLayer().Save()
        print("\nStage saved.")
    else:
        print("\nSAVE_STAGE is False -- edits made in memory only.")

    print("Next: run build_vica_ros_graphs.py, then Play and check /cmd_vel drives the robot.")


main()
