-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_frame",
  published_frame = "base_link",
  odom_frame = "odom_cart",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
-- num_subdivisions_per_laser_scan = 10,

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2

-------- tune parameters in the trajectory builder 2D --------

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- can limit the search space of the online correlative scan matcher when IMU is available
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.) --  default=math.rad(20.),

-- Set min_z and max_z to convert 3D point cloud in 2D localization
TRAJECTORY_BUILDER_2D.min_z = 0.5
TRAJECTORY_BUILDER_2D.max_z = 1.8

-- filter point cloud to improve the realtime computation performance
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1 --default=0.025

-- increase this value to put high reliability to lidar data
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0 --default 1.0

-------- tune parameters in the pose graph --------

-- reduce the frequency of pose graph optimization
-- POSE_GRAPH.optimize_every_n_nodes = 20 - default 20

-- reduce the frequency of local constraint search
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.0 -- default=15.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 -- default=0.3

-- disable global constraint search
POSE_GRAPH.global_sampling_ratio =  0.0 --default=0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 360000 --default=10

-- decrease these values when the constraint builder cannot find matches between a new trajectory and an old trajectory
-- POSE_GRAPH.constraint_builder.min_score = 0.3 -- cartographer default 0.55
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.35 -- cartographer default 0.6

return options
