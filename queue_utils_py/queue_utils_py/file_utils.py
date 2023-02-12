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

import json
import os
import yaml

from geometry_msgs.msg import Point, Pose
import resource_retriever
from tf_transformations import quaternion_from_euler


class QueueUtilError(Exception):
    pass


def get_filename(string):
    if string.startswith("~/"):
        return os.path.expanduser(string)
    elif string.startswith("package://"):
        return resource_retriever.get_filename(string, use_protocol=False)
    else:
        return string


def load_queue_path_file(queue_path_file):
    """
    load annotation file for queue expected path which is created by "ros_map_annotator"
    Args:
        queue_path_file: txt file for representing queue expected path created by "ros_map_annotator"
    Returns:
        queue_path_pose_array: array of geometry_msgs.msg.Pose representing queue expected path
    """
    if not os.path.exists(queue_path_file):
        raise QueueUtilError("queue path file does not exist : " + queue_path_file)

    with open(queue_path_file, 'r') as f:
        queue_path_json = json.load(f)
    print("queue_path_file contents = " + json.dumps(queue_path_json, sort_keys=True, indent=4))

    queue_path_pose_array = []
    queue_path_poses_json = queue_path_json["poses"]
    for pose in queue_path_poses_json:
        pose_msg = Pose()
        pose_msg.position = Point(x=float(pose["pose"]["x"]), y=float(pose["pose"]["y"]), z=0.0)
        pose_orientation_quat = quaternion_from_euler(0.0, 0.0, float(pose["pose"]["orientation"]))
        pose_msg.orientation.x = pose_orientation_quat[0]
        pose_msg.orientation.y = pose_orientation_quat[1]
        pose_msg.orientation.z = pose_orientation_quat[2]
        pose_msg.orientation.w = pose_orientation_quat[3]
        queue_path_pose_array.append(pose_msg)
    return queue_path_pose_array


def load_queue_obstacle_file(queue_obstacle_file):
    if not os.path.exists(queue_obstacle_file):
        raise QueueUtilError("queue obstacle file does not exist: "+queue_obstacle_file)

    with open(queue_obstacle_file, 'r') as f:
        queue_obstacle_json = json.load(f)
    print("queue_obstacle_file contents = " + json.dumps(queue_obstacle_json, sort_keys=True, indent=4))

    queue_obstacle_points_json = queue_obstacle_json["points"]
    queue_obstacle_point_array = []
    for point in queue_obstacle_points_json:
        queue_obstacle_point_array.append((float(point["x"]), float(point["y"])))
    return queue_obstacle_point_array


def load_queue_annotation_list_file(queue_list_file):
    if not os.path.exists(queue_list_file):
        raise QueueUtilError("queue annotation list file does not exist: "+queue_list_file)

    with open(queue_list_file, 'r') as f:
        queue_list_data = yaml.safe_load(f)

    if "queue_list" not in queue_list_data:
        raise QueueUtilError("queue annotation list file does not have 'queue_list' key")

    queue_annotation_frame_id_dict = {}
    for frame_queue_list in queue_list_data["queue_list"]:
        if "frame_id" not in frame_queue_list or "frame_queue_list" not in frame_queue_list:
            raise QueueUtilError("invalid frame queue list data = " + str(frame_queue_list))

        frame_id = frame_queue_list["frame_id"]
        queue_annotation_frame_id_dict[frame_id] = []

        for queue in frame_queue_list["frame_queue_list"]:
            if "queue_expected_path_filename" not in queue or "queue_obstacle_polygon_filename" not in queue:
                raise QueueUtilError("invalid queue data = " + str(queue))

            queue_annotation = {}

            queue_expected_path_filename = os.path.expanduser(queue["queue_expected_path_filename"])
            queue_annotation["queue_expected_path"] = load_queue_path_file(get_filename(queue_expected_path_filename))

            queue_obstacle_polygon_filename = os.path.expanduser(queue["queue_obstacle_polygon_filename"])
            queue_annotation["queue_obstacle_polygon"] = load_queue_obstacle_file(get_filename(queue_obstacle_polygon_filename))

            queue_annotation_frame_id_dict[frame_id].append(queue_annotation)

    return queue_annotation_frame_id_dict
