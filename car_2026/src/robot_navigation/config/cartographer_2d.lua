include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- =========================================================
  -- TF frames
  -- =========================================================

  map_frame = "map",

  -- IMU frame is used as Cartographer tracking frame.
  tracking_frame = "imu_link",

  -- IMPORTANT:
  -- EKF already publishes:
  --
  --   odom -> base_footprint
  --
  -- Therefore Cartographer should publish:
  --
  --   map -> odom
  --
  published_frame = "odom",

  odom_frame = "odom",

  -- Do NOT let Cartographer create another odom frame.
  provide_odom_frame = false,

  publish_frame_projected_to_2d = false,


  -- =========================================================
  -- Sensors
  -- =========================================================

  use_odometry = true,

  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,

  num_point_clouds = 0,


  -- =========================================================
  -- Timing
  -- =========================================================

  lookup_transform_timeout_sec = 0.2,

  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}


-- ============================================================
-- 2D SLAM
-- ============================================================

MAP_BUILDER.use_trajectory_builder_2d = true


-- ============================================================
-- Laser
-- ============================================================

TRAJECTORY_BUILDER_2D.min_range = 0.10

-- Gazebo LiDAR currently supports up to 10 m.
-- The field is only about 4.2 m, so 8 m is sufficient.
TRAJECTORY_BUILDER_2D.max_range = 8.0

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0


-- ============================================================
-- IMU
-- ============================================================

TRAJECTORY_BUILDER_2D.use_imu_data = true


-- ============================================================
-- Submaps
-- ============================================================

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35


-- ============================================================
-- Scan matching
-- ============================================================

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =
    0.1

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight =
    10.0

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =
    0.1


-- ============================================================
-- Motion filter
-- ============================================================

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians =
    math.rad(0.2)


-- ============================================================
-- Pose graph
-- ============================================================

POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.constraint_builder.min_score = 0.60

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70


return options
