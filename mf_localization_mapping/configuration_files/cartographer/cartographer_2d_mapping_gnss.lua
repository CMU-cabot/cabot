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

include "cartographer_2d_mapping.lua"

options.use_nav_sat = true
options.fixed_frame_pose_sampling_ratio = 1.0 -- 10 Hz * sampling_ratio = 1.0 Hz

TRAJECTORY_BUILDER.collate_fixed_frame = false -- default: true
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1 -- default: 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 0.0 -- GPS provides no orientation.
POSE_GRAPH.optimization_problem.fixed_frame_pose_use_tolerant_loss = true -- use tolerant loss

return options
