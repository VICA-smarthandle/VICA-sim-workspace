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

Placement is read out of the stage rather than guessed: the camera sits 20 mm
forward of camera_link, which is the D455 asset's own mounting offset, and the
lidar sits at laser_frame's offset -- but hung off base_link rather than off
laser_frame itself, for a reason that took a long time to find and is written
up above _attach_lidar.

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

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


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
# The asset is an S2E, but its range is not baked in: the OmniLidar prim
# carries omni:sensor:Core:nearRangeM and farRangeM as ordinary USD attributes,
# so they can be overridden to the A2's limits. The asset stays an S2E in every
# other respect -- 32 kHz firing rate, 10 Hz spin -- which is close enough to
# the A2's 8 kHz that the distinction does not survive a 0.05 m costmap.
#
# Ranges are what actually matter downstream: they set /scan's range_min and
# range_max, which is what cartographer and amcl clip against.
LIDAR_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/6.0/Isaac/Sensors/Slamtec/RPLIDAR_S2E/Slamtec_RPLIDAR_S2E.usd"
)
LIDAR_PRIM_NAME = "RPLIDAR_S2E"
# The lidar hangs off base_link under this Xform rather than off laser_frame.
# See _attach_lidar for why.
LIDAR_MOUNT_NAME = "lidar_mount"

# RPLIDAR A2, matching config/vica_2d.lua and the amcl block. The asset ships
# 0.05 / 30.0. min 0.2 covers both A2 variants (A2M8 0.15 m, A2M12 0.2 m).
LIDAR_NEAR_RANGE_M = 0.2
LIDAR_FAR_RANGE_M = 12.0

# And the A2's firing rate, for the same reason.
#
# The S2E fires at 32 kHz. Spinning at 10 Hz that is 3200 points per
# revolution, an angular resolution of 0.11 deg; the A2 fires at 8 kHz for 800
# points and 0.45 deg. Left alone, the simulated scan is four times denser than
# the robot's, and at 5 m the beams land 1 cm apart here against 4 cm there.
#
# That gap points the wrong way for what this simulator is for. Chair legs,
# poles and door frames read reliably in simulation and get missed between
# beams on the robot, so a narrow gap that navigates cleanly here says nothing
# about the same gap in a corridor. Better to simulate the sensor that exists.
#
# It also fixes the size of /scan's ranges array, which is what anything doing
# arithmetic on beam indices depends on.
LIDAR_FIRING_RATE_HZ = 8000

# Read back out of the previous stage rather than assumed.
CAMERA_OFFSET = Gf.Vec3d(0.02, 0.0, 0.0)

# --------------------------------------------------------------------------
# Camera optics
# --------------------------------------------------------------------------
# The RSD455 asset gives all four of its cameras the colour module's lens --
# focalLength 1.93, apertures 3.896 x 2.453, so 90.5 x 64.9 degrees. That is
# right for the D455's colour stream and wrong for its depth stream, which is
# 87 x 58 through a different imager. Camera_Pseudo_Depth inherits it anyway.
#
# It matters because the depth camera is what sees below the lidar plane. The
# robot's own nvblox notes reason from "보는 범위 = 카메라 수직 FOV 58°", and a
# simulated camera seeing 64.9 -- or 74.3 after the square-pixel forcing below
# -- keeps obstacles in view through turns that would lose them on the robot.
# The 2026-07-28 collision there was exactly an obstacle leaving view mid-turn.
#
# Isaac derives camera_info as
#
#     fx = focalLength * width / horizontalAperture
#     fy = focalLength * height / verticalAperture
#
# then forces fy to fx because the renderer assumes square pixels. So the
# aperture ratio has to equal the render resolution ratio or the vertical field
# of view silently becomes whatever the horizontal one implies. At 640x480
# against the asset's 1.588 aperture ratio, 64.9 became 74.3.
#
# Depth: hold vertical at 58 and let the aperture ratio follow 848x480, the
# D455's native depth resolution. Horizontal lands at 88.8, inside the +/-3 the
# datasheet allows, and nothing is forced.
DEPTH_VERTICAL_FOV_DEG = 58.0
DEPTH_RESOLUTION = (848, 480)
# Colour: the asset's optics are already right; only the resolution ratio was
# wrong. 640x400 is the OV9782's 1.6 aspect, so 90.5 x 64.9 survives.
COLOR_RESOLUTION = (640, 400)

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


def _strip_physics(stage, root):
    """Remove rigid-body and collision schemas from an attached sensor asset.

    NVIDIA ships these sensors as standalone props, so rsd455.usd applies
    RigidBodyAPI to its own root. Referenced under camera_link it becomes a
    dynamic body that no joint holds: press Play and the camera drops away from
    the robot and keeps going. It looks like the mount is broken.

    Worse than the falling is where it falls from. A loose rigid body nested
    inside a link of an articulation is not something PhysX handles gracefully,
    and the articulation misbehaves around it -- a robot that drives forwards
    and backwards but will not turn is the shape that took here.

    A sensor should be neither. It rides on the link it is mounted to and
    reports what it sees; it has no business having mass or a collider.
    """
    removed = []
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(root):
            continue
        for api, name in (
            (UsdPhysics.RigidBodyAPI, "RigidBodyAPI"),
            (UsdPhysics.CollisionAPI, "CollisionAPI"),
            (UsdPhysics.ArticulationRootAPI, "ArticulationRootAPI"),
        ):
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
                # RemoveAPI does not compose through a reference, so disable it
                # by attribute as well.
                for attr_name in (
                    "physics:rigidBodyEnabled",
                    "physics:collisionEnabled",
                ):
                    attr = prim.GetAttribute(attr_name)
                    if attr:
                        attr.Set(False)
                removed.append(f"{prim.GetName()}:{name}")
    if removed:
        print(f"    physics stripped from sensor: {', '.join(removed)}")
    return removed


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


def _set_depth_optics(stage, camera_root):
    """Give the depth camera the depth module's field of view.

    Only the depth camera is touched. The colour camera already carries the
    right lens for what it is.
    """
    import math

    w, h = DEPTH_RESOLUTION
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(camera_root) or not prim.IsA(UsdGeom.Camera):
            continue
        if "Pseudo_Depth" not in prim.GetName():
            continue
        cam = UsdGeom.Camera(prim)
        focal = cam.GetFocalLengthAttr().Get()
        v_ap = 2.0 * focal * math.tan(math.radians(DEPTH_VERTICAL_FOV_DEG / 2.0))
        # Aperture ratio equal to the resolution ratio, so fy comes out equal
        # to fx and the square-pixel forcing has nothing to change.
        h_ap = v_ap * w / h
        before_v = cam.GetVerticalApertureAttr().Get()
        before_h = cam.GetHorizontalApertureAttr().Get()
        cam.CreateVerticalApertureAttr().Set(v_ap)
        cam.CreateHorizontalApertureAttr().Set(h_ap)
        fx = focal * w / h_ap
        fy = focal * h / v_ap
        print(f"    optics  -> {prim.GetName()}")
        print(f"        aperture h {before_h:.4f} -> {h_ap:.4f}   "
              f"v {before_v:.4f} -> {v_ap:.4f}")
        print(f"        at {w}x{h}: fx {fx:.1f} fy {fy:.1f}   "
              f"H {2 * math.degrees(math.atan(w / 2 / fx)):.1f} deg  "
              f"V {2 * math.degrees(math.atan(h / 2 / fy)):.1f} deg")
        return True
    print("    WARNING: no Pseudo_Depth camera found, optics left as shipped")
    return False


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
        _strip_physics(stage, path)
        _set_depth_optics(stage, path)
        return path

    xform = UsdGeom.Xform.Define(stage, path)
    xform.GetPrim().GetReferences().AddReference(RSD455_URL)
    xform.AddTranslateOp().Set(CAMERA_OFFSET)
    print(f"    camera  -> {path}  offset {tuple(CAMERA_OFFSET)}")
    _strip_physics(stage, path)
    _set_depth_optics(stage, path)
    return path


def _attach_lidar(stage, base_link):
    """Mount the lidar on base_link, at laser_frame's offset.

    Not under laser_frame, which is the obvious place and does not work. The
    importer marks link prims carrying a mesh as instanceable, and laser_frame
    carries the lidar's visual. Clearing that flag is enough for USD to let a
    child be authored -- the prim then reports instanceable False, IsInstance
    False, IsInstanceProxy False -- but not enough for the RTX sensor pipeline,
    which produces no scan at all from a lidar parented there. The topic is
    advertised, the graph runs, and nothing is ever published.

    Nothing in a static comparison shows this. The lidar prim's attributes,
    applied schemas and flags all match a lidar that works, and the same graph
    publishes when pointed at one. It took running the same graph against three
    lidars -- one on a working robot, one on nothing at all, one here -- to
    place the fault on the parent rather than the sensor.

    Mounting on base_link instead costs nothing: laser_frame is fixed to
    base_link, so a copy of its transform puts the sensor in the same place,
    and /scan still carries frame_id laser_frame, which TF resolves from the
    URDF as before.
    """
    laser_frame = f"{base_link}/laser_frame"
    frame_prim = stage.GetPrimAtPath(laser_frame)
    if not frame_prim:
        raise RuntimeError(f"No laser_frame at {laser_frame}")

    # An earlier run of this script put a lidar under laser_frame. Leaving it
    # there is worse than the original bug: two OmniLidar prims in one stage and
    # neither publishes, so the fix below would look like it had not worked.
    stale = stage.GetPrimAtPath(f"{laser_frame}/{LIDAR_PRIM_NAME}")
    if stale and stale.IsActive():
        stale.SetActive(False)
        print(f"    stale   -> {laser_frame}/{LIDAR_PRIM_NAME} deactivated")

    # From laser_joint, not from laser_frame's transform.
    #
    # A URDF import leaves link prims carrying their visual mesh offset -- for
    # laser_frame that is (0, 0, -0.041) -- while the joints carry the
    # kinematics, and PhysX places the links from those at Play. Reading the
    # prim gives the mesh offset and puts the lidar inside the chassis.
    offset = None
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        targets = [t.name for t in joint.GetBody1Rel().GetTargets()]
        if "laser_frame" in targets:
            offset = joint.GetLocalPos0Attr().Get()
            print(f"    offset  <- {prim.GetName()}.localPos0 = "
                  f"{tuple(round(v, 4) for v in offset)}")
            break
    if offset is None:
        raise RuntimeError(
            "No joint found with laser_frame as body1; cannot place the lidar."
        )

    parent = f"{base_link}/{LIDAR_MOUNT_NAME}"
    xf = UsdGeom.Xform.Define(stage, parent)
    # Rewrite rather than create-if-missing: a re-run has to be able to correct
    # a mount placed by an earlier version of this script.
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*offset))
    print(f"    mount   -> {parent}  at laser_frame's offset "
          f"{tuple(round(v, 4) for v in offset)}")

    path = f"{parent}/{LIDAR_PRIM_NAME}"
    if stage.GetPrimAtPath(path):
        print(f"    lidar already present at {path}")
        # Still reapply the range. It is a setting, not part of creating the
        # prim, and skipping it on a re-run is how a stage ends up with the
        # sensor in place and the shipped 30 m range nobody notices.
        _set_lidar_range(stage, path)
        return path

    # Referenced, not created through IsaacSensorCreateRtxLidar: the command
    # only accepts configs from its hardcoded table, and referencing the asset
    # is what the previous stage did anyway. Sits on laser_frame with no offset
    # and no rotation, read back out of that stage rather than guessed.
    xform = UsdGeom.Xform.Define(stage, path)
    xform.GetPrim().GetReferences().AddReference(LIDAR_URL)
    print(f"    lidar   -> {path}  (RPLIDAR S2E asset)")
    _set_lidar_range(stage, path)
    return path


def _set_lidar_range(stage, lidar_root):
    """Override the sensor's range and firing rate to the A2's.

    Applied in place on the OmniLidar prim.

    The referenced asset has to be composed before this can find the prim, so it
    runs after AddReference rather than alongside it. Returns quietly if the
    reference has not resolved -- both sensor assets are fetched over the
    network, and an offline run has nothing to override.
    """
    for prim in stage.Traverse():
        if prim.GetTypeName() != "OmniLidar":
            continue
        if not str(prim.GetPath()).startswith(lidar_root):
            continue
        for attr_name, value in (
            ("omni:sensor:Core:nearRangeM", LIDAR_NEAR_RANGE_M),
            ("omni:sensor:Core:farRangeM", LIDAR_FAR_RANGE_M),
            ("omni:sensor:Core:patternFiringRateHz", LIDAR_FIRING_RATE_HZ),
        ):
            attr = prim.GetAttribute(attr_name)
            if not attr:
                print(f"    WARNING: {attr_name} absent, left as shipped")
                continue
            before = attr.Get()
            attr.Set(value)
            print(f"    sensor  -> {attr_name.split(':')[-1]}: {before} -> {value}")
        return
    print("    WARNING: no OmniLidar under the reference, left as shipped")


# --------------------------------------------------------------------------
# Ultrasonic probes
# --------------------------------------------------------------------------
# Two DYP-A22 on the front bumper, fitted to the robot 2026-08-31, at ankle
# height where neither the lidar plane (0.382) nor the costmap's depth band
# (0.30 to 1.05) has anything to say.
#
# Modelled as IsaacRaycastSensor rather than an RTX lidar. An ultrasonic
# returns one distance, not a scan, and the RTX path needs a config from a
# hardcoded table this sensor is not in. The raycast reads the same PhysX
# scene the wheels roll on, which is what a range reading should agree with.
#
# The cone is approximated by a small fan rather than one ray. A single ray
# through the middle misses an obstacle its own width off axis, which is the
# case the probes exist to catch; DYP-A22 opens about 60 degrees and the fan
# below covers the middle 30 of it. nav2's RangeSensorLayer takes the minimum
# anyway, so more rays only sharpen the same number.
USONIC_NAMES = ("usonic_front_left", "usonic_front_right")
USONIC_MIN_RANGE_M = 0.02
USONIC_MAX_RANGE_M = 4.0
USONIC_FAN_DEG = 30.0
USONIC_FAN_RAYS = 7


def _attach_usonic(stage, base_link):
    """Mount a raycast sensor at each ultrasonic frame's joint offset.

    On base_link, not under the usonic_* prims, for the reason _attach_lidar
    records at length: the importer marks link prims carrying a mesh as
    instanceable, and a sensor parented there is advertised and never fires.
    Both probes now carry a visual cylinder, so both would hit it.
    """
    made = []
    for name in USONIC_NAMES:
        frame = f"{base_link}/{name}"
        if not stage.GetPrimAtPath(frame):
            print(f"    SKIPPED {name}: no such link")
            continue

        offset = None
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            if name in [t.name for t in joint.GetBody1Rel().GetTargets()]:
                offset = joint.GetLocalPos0Attr().Get()
                break
        if offset is None:
            print(f"    SKIPPED {name}: no joint places it")
            continue

        mount = f"{base_link}/{name}_mount"
        xf = UsdGeom.Xform.Define(stage, mount)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*offset))

        path = f"{mount}/{name}_ray"
        prim = stage.GetPrimAtPath(path)
        if not prim:
            prim = stage.DefinePrim(path, "IsaacRaycastSensor")
        # Written every run, not only on create: these are settings, and a
        # re-run has to be able to correct a probe configured by an older
        # version of this file.
        for attr, kind, value in (
            ("minRange", Sdf.ValueTypeNames.Float, USONIC_MIN_RANGE_M),
            ("maxRange", Sdf.ValueTypeNames.Float, USONIC_MAX_RANGE_M),
            ("drawPoints", Sdf.ValueTypeNames.Bool, False),
            ("drawLines", Sdf.ValueTypeNames.Bool, False),
        ):
            a = prim.GetAttribute(attr) or prim.CreateAttribute(attr, kind)
            a.Set(value)
        # The fan, as yaw offsets about the mount's z.
        half = USONIC_FAN_DEG / 2.0
        step = USONIC_FAN_DEG / max(1, USONIC_FAN_RAYS - 1)
        angles = [(-half + i * step) for i in range(USONIC_FAN_RAYS)]
        a = (prim.GetAttribute("beamAngles")
             or prim.CreateAttribute("beamAngles", Sdf.ValueTypeNames.FloatArray))
        a.Set([float(v) for v in angles])
        print(f"    usonic  -> {path}  at {tuple(round(v, 4) for v in offset)}  "
              f"{USONIC_FAN_RAYS} rays over {USONIC_FAN_DEG:.0f} deg")
        made.append(path)
    return made


def main():
    _require_stopped_timeline()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    base_link = _find_base_link(stage)

    print("=== attaching sensors")
    camera_path = _attach_camera(stage, base_link)
    lidar_path = _attach_lidar(stage, base_link)
    usonic_paths = _attach_usonic(stage, base_link)

    if SAVE_STAGE:
        stage.GetRootLayer().Save()

    print(f"\nbase link     : {base_link}")
    print(f"camera        : {camera_path}")
    print(f"lidar         : {lidar_path}")
    print(f"ultrasonic    : {usonic_paths or 'none'}")
    if SAVE_STAGE:
        print("Stage saved.")
    else:
        print("SAVE_STAGE is False -- stage NOT written.")

    print("\nNext: build_vica_ros_graphs.py against this stage.")


main()
