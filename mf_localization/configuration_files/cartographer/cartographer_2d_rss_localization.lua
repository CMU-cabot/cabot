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

-- for initial localization

-- reduce the number of points in scan to reduce the computational cost
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.2 --default=0.025

-- enlarge the search space of the constraint builder for initial localization
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0 --default 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 30.0  --default 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = 3.14 --default math.rad(30.)

-- disable global constraint search when coarse initial location can be given by an external localizer.
POSE_GRAPH.global_sampling_ratio =  0.000 --default=0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 360000 --default 10

-- for fast localization
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05 --default=0.3

-- increase these values when the constraint builder find wrong matches between a new trajectory and an old trajectory
POSE_GRAPH.constraint_builder.min_score = 0.5 -- cartographer default 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55 -- cartographer default 0.6

return options
