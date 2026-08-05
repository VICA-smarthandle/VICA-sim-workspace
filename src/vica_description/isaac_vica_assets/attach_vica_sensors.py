"""Attach the lidar and the depth camera to an imported VICA.

Run from Isaac's Script Editor, or headless against a stage. Timeline stopped.

Why this exists
---------------
The URDF importer creates links, joints and collision, and no sensors. A URDF
has nowhere to say "there is an RTX lidar here" -- laser_frame and camera_link
are just frames. So every re-import produces a robot that is blind, and the
sensors have to be put back by hand.

They were, once, in the GUI. That is the same trap as the Action Graphs and the
joint drives: work that lives only in a stage file, is lost on the next import,
and leaves nothing behind saying what was lost. Running the graph builder on a
freshly imported robot says it plainly:

    SKIPPED: ROS_Lidar (no OmniLidar prim under the robot)
    SKIPPED: ROS_Camera (RSD455 colour/depth prims not found)

Placement matches what the previous stage had, read back out of it rather than
guessed: the lidar sits on laser_frame with no offset, and the camera 20 mm
forward of camera_link, which is the D455 asset's own mounting offset.

The lidar is an S2E, as it was before, and the robot's is an A2. That gap
cannot be closed here; see the note above LIDAR_URL for why, and for where it
is absorbed instead.

Run this against the robot asset (robot/vica/vica.usda), not against a composed
stage. Sensors belong to the robot, so every stage referencing it gets them --
and a composed stage cannot take them anyway, since the robot arrives there as
an instance and USD will not author inside one.

Order: export_isaac_urdf.sh -> URDF importer -> fixup_vica_usd_joints.py ->
this -> build_vica_hospital_stage.py -> build_vica_ros_graphs.py.
"""

import omni.kit.commands
import omni.timeline
import omni.usd

from pxr import Gf, Sdf, Usd, UsdGeom


# --------------------------------------------------------------------------
# Assets
# --------------------------------------------------------------------------
# Fetched over the network, exactly as the previous stage did. Isaac ships no
# local copy of either, so an offline session loses both sensors -- which is
# worth knowing before blaming the scripts.
RSD455_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/6.0/Isaac/Sensors/RealSense/D455/rsd455.usd"
)

# SLAMTEC RPLIDAR S2E, the same asset the previous stage referenced.
#
# The robot runs an A2, and there is no way to say so here. Isaac 6.0 resolves
# RTX lidars from a hardcoded table of sensor USDs (SUPPORTED_LIDAR_CONFIGS in
# isaacsim.sensors.experimental.rtx) and ships no A2. The JSON profile system
# that would accept a custom sensor description belongs to the deprecated
# camera-based lidar path, so authoring an A2 profile does not reach this one:
# creating a lidar with a custom config name fails with
# "Config 'X' not found for OmniLidar".
#
# What that leaves is a simulated sensor that ranges to 30 m against hardware
# that ranges to 12 m. It is handled downstream rather than at the source --
# cartographer's max_range and amcl's laser_max_range are both 12.0, so returns
# past that are discarded before they reach a map or a pose. The residue is
# that /scan advertises range_max 30.0, so anything reading that field rather
# than filtering by it will believe the wrong sensor.
LIDAR_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/6.0/Isaac/Sensors/Slamtec/RPLIDAR_S2E/Slamtec_RPLIDAR_S2E.usd"
)
LIDAR_PRIM_NAME = "RPLIDAR_S2E"

# Read back out of the previous stage rather than assumed.
CAMERA_OFFSET = Gf.Vec3d(0.02, 0.0, 0.0)

ARTICULATION_SUFFIX = "/Geometry/base_footprint/base_link"

SAVE_STAGE = True


def _require_stopped_timeline():
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Stop the Timeline before attaching sensors.")


def _find_base_link(stage):
    matches = [
        p for p in stage.Traverse()
        if str(p.GetPath()).endswith(ARTICULATION_SUFFIX)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one prim ending in {ARTICULATION_SUFFIX}, "
            f"found {len(matches)}: {[str(m.GetPath()) for m in matches]}"
        )
    return str(matches[0].GetPath())


def _make_authorable(stage, path):
    """Clear the instanceable flag so a child can be added under *path*.

    The importer marks every link prim that carries a mesh as instanceable, and
    USD refuses to author anything beneath an instance: "authoring to an
    instance proxy is not allowed". laser_frame is one of those, because it
    still has the lidar's visual mesh; camera_link is not, because its visual
    is a primitive box. So the lidar hits this and the camera does not.

    Nothing is lost by clearing it. Instancing pays off when a prim appears
    many times over -- these appear exactly once each.
    """
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsInstanceable():
        prim.SetInstanceable(False)
        print(f"    {path.rsplit('/', 1)[-1]}: instanceable cleared to allow a sensor child")
    return prim


def _attach_camera(stage, base_link):
    """Reference the D455 under camera_link.

    The prim has to be called rsd455: build_vica_ros_graphs.py looks for the
    colour and depth cameras at rsd455/RSD455/Camera_*, which is the asset's
    own internal layout.
    """
    _make_authorable(stage, f"{base_link}/camera_link")
    path = f"{base_link}/camera_link/rsd455"
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        print(f"    camera already present at {path}")
        return path

    xform = UsdGeom.Xform.Define(stage, path)
    xform.GetPrim().GetReferences().AddReference(RSD455_URL)
    xform.AddTranslateOp().Set(CAMERA_OFFSET)
    print(f"    camera  -> {path}  offset {tuple(CAMERA_OFFSET)}")
    return path


def _attach_lidar(stage, base_link):
    parent = f"{base_link}/laser_frame"
    if not stage.GetPrimAtPath(parent):
        raise RuntimeError(f"No laser_frame at {parent}")

    _make_authorable(stage, parent)

    path = f"{parent}/{LIDAR_PRIM_NAME}"
    if stage.GetPrimAtPath(path):
        print(f"    lidar already present at {path}")
        return path

    # Referenced, not created through IsaacSensorCreateRtxLidar: the command
    # only accepts configs from its hardcoded table, and referencing the asset
    # is what the previous stage did anyway. Sits on laser_frame with no offset
    # and no rotation, read back out of that stage rather than guessed.
    xform = UsdGeom.Xform.Define(stage, path)
    xform.GetPrim().GetReferences().AddReference(LIDAR_URL)
    print(f"    lidar   -> {path}  (RPLIDAR S2E asset)")
    return path


def main():
    _require_stopped_timeline()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    base_link = _find_base_link(stage)

    print("=== attaching sensors")
    camera_path = _attach_camera(stage, base_link)
    lidar_path = _attach_lidar(stage, base_link)

    if SAVE_STAGE:
        stage.GetRootLayer().Save()

    print(f"\nbase link     : {base_link}")
    print(f"camera        : {camera_path}")
    print(f"lidar         : {lidar_path}")
    if SAVE_STAGE:
        print("Stage saved.")
    else:
        print("SAVE_STAGE is False -- stage NOT written.")

    print("\nNext: build_vica_ros_graphs.py against this stage.")


main()
