-- Copyright (c) 2021  IBM Corporation
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.

include "cartographer_2d_localization.lua"

-- for tracking

-- reduce the number of points in scan to reduce the computational cost
TRAJECTORY_BUILDER_2D.max_range = 15.0
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1 --default=0.025

-- reduce the frequency that nodes are added to submaps
--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0 -- default 5.0
--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 -- default 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5.0) -- default math.rad(1.0)

-- change the search space of the constraint builder
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.0  --default 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3 --default 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.) --default math.rad(30.)

-- disable global constraint search when coarse initial location can be given by an external localizer.
POSE_GRAPH.global_sampling_ratio =  0.000 --default=0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 360000 --default 10

-- for fast localization
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05 --default=0.3

return options
