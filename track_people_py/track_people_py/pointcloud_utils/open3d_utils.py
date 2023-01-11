# Copyright (c) 2021  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import cv2
import numpy as np
import open3d as o3d


def generate_pointcloud(image_width, image_height, focal_length, center_x, center_y, rgb, depth, remove_noise=False,
                        depth_unit_meter=True):
    if rgb.shape != depth.shape:
        depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))
    depth = np.float32(depth)

    # if you use zed-ros-wrapper, unit is already meter
    if not depth_unit_meter:
        depth = depth/1000.0

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)), o3d.geometry.Image(depth), convert_rgb_to_intensity=False)
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(image_width, image_height, focal_length, focal_length, center_x, center_y)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

    # To avoid problem of Open3D
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)*1000.0)

    if remove_noise:
        uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
        cl, ind = uni_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.2)
        inlier_cloud = uni_down_pcd.select_by_index(ind)
        return inlier_cloud.points, inlier_cloud.colors
    else:
        return pcd.points, pcd.colors
