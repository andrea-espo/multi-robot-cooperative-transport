include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "tb3_3/map",
  odom_frame = "tb3_3/odom",

  tracking_frame  = "tb3_3/base_footprint",
  published_frame = "tb3_3/base_footprint",

  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec   = 0.2,
  submap_publish_period_sec      = 0.5,
  pose_publish_period_sec        = 0.02,
  trajectory_publish_period_sec  = 0.02,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Sensore (LDS-01/LD08)
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- Niente IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Matching più robusto indoor
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 35.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 120.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 8

-- Filtro di moto più stretto
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.06
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(0.2)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.5

-- Submap più densa
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100

-- Pose-graph più prudente
POSE_GRAPH.optimize_every_n_nodes = 120
POSE_GRAPH.constraint_builder.min_score = 0.80
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85
POSE_GRAPH.optimization_problem.huber_scale = 5.0
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 25

return options

