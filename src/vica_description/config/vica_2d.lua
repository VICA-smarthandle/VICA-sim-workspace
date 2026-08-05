-- VICA Cartographer 2D SLAM -- Isaac Sim
--
-- Ported from the physical robot (VICA-smarthandle: vica_cartographer/config/
-- vica_2d.lua). The mapping stack is deliberately the same in both places, so
-- that a map built here and a map built on the robot fail in the same ways and
-- the tuning transfers in either direction.
--
-- TF ownership:
--   Cartographer: map -> odom
--   Isaac:        odom -> base_footprint   (ROS_Odometry action graph)
--   URDF/RSP:     base_footprint -> base_link -> laser_frame/camera_link
--
-- The robot fills the odom -> base_footprint slot with an EKF; Isaac's odometry
-- graph fills it here. Cartographer cannot tell the difference, which is why
-- provide_odom_frame, published_frame and use_odometry all carry over unchanged.
--
-- One difference worth remembering: Isaac's odometry is ground truth. The
-- odometry weights near the bottom of this file express how much to trust a
-- drifting wheel EKF, so in simulation they are pessimistic. They are left at
-- the robot's values anyway -- a map that only holds together because the
-- odometry was perfect would tell us nothing about the real robot.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",

  -- Isaac's ROS_Odometry graph provides odom -> base_footprint.
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 10e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- No IMU on either side: the robot maps without one, and Isaac's graphs publish
-- no IMU topic.
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- RPLIDAR A2 range limits.
--
-- The robot's config still carries YDLIDAR G2 numbers (0.15 / 8.0) from before
-- the sensor swap, so these are not a straight copy -- they are the A2's own
-- limits and the robot's file needs the same correction.
--
-- min_range 0.2 covers both A2 variants: A2M8 specifies 0.15 m, A2M12 0.2 m.
-- Erring high costs 5 cm of blind zone; erring low lets sub-minimum returns
-- into the map as phantom obstacles right against the chassis.
--
-- max_range 12.0 is the A2's rated range. The old 8.0 was a deliberate clip of
-- the G2's 12 m spec down to what it read reliably indoors; if walls here turn
-- out to be glass-heavy and the map grows spurious depth, clip this to 10.0
-- rather than reaching for the scan matcher weights.
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.5

-- Keep online scan matching enabled to compensate small wheel odom errors.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0

-- Low-speed indoor mapping update thresholds.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Conservative loop-closure values for repeated indoor corridors.
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

-- If maps smear, verify in this order:
--   /scan hz, /odom hz, odom -> base_footprint,
--   base_footprint -> base_link, base_link -> laser_frame,
--   wheel_radius_m, wheel_base_m, motor direction, timestamps.
--
-- In simulation add two more before any of the above, because both have
-- silently produced a wrong robot here already:
--   the drive wheel separation in the imported USD (0.364, not 0.293), and
--   the RTX lidar profile on the laser prim (must be an A2, not whatever
--   profile the sensor was created with).

return options
