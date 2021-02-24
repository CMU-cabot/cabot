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

include "cartographer_2d.lua"

-- For pure localization
-- TRAJECTORY_BUILDER.pure_localization = true -- for v1.0.0
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3 --default=3
}

-- for frequent localization
POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.global_sampling_ratio =  0.0003 --default=0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1 --default=0.3

-- increase these values when the constraint builder find wrong matches between a new trajectory and an old trajectory
--POSE_GRAPH.constraint_builder.min_score = 0.3 -- cartographer default 0.55
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.35 -- cartographer default 0.6

return options
