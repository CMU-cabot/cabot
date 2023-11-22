#!/usr/bin/env python3

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

import copy
import math
import yaml

import rospy
import rosbag
import std_msgs.msg
from geometry_msgs.msg import Point, Pose
import matplotlib
import nav_msgs.msg
import tf
from tf.transformations import euler_from_quaternion
import tf2_ros
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray

from explore_manager_node import ExploreManagerNode
from route_explore_utils import geometry_utils, visualize_utils

#roslaunch route_explore explore_evaluation.launch bagfile:=explore-2021-12-13-22-58-57.bag annotationfile:=nsh-4f-annotation.yaml debug:=true

select_goal_quat_forward_idx = 0
select_goal_quat_right_idx = 1
select_goal_quat_left_idx = 2
select_goal_quat_backward_idx = 3

angle_threshold = math.pi * 15 / 180 
straight_corridor_threshold = math.pi * 170 / 180 

class msgs():
    def __init__(self,mapmsg,scanmsg,odommsg,worldposemsg):
        self.mapmsg = mapmsg
        self.scanmsg = scanmsg
        self.odommsg = odommsg
        self.worldposemsg = worldposemsg

def main():
    global current_idx
    current_idx = 0

    global next_line
    next_line = False

    global allow_input
    allow_input = False

    recorded_time = set()

    rospy.init_node('explore_evaluation', log_level=rospy.FATAL)

    use_scan_topic_free_area = rospy.get_param("~use_scan_topic_free_area", True)
    use_dt_polyhedron_sensor_range = rospy.get_param("~use_dt_polyhedron_sensor_range", True)
    default_polyhedron_sensor_range = rospy.get_param("~default_polyhedron_sensor_range", 8.0)
    dt_polyhedron_closest_node_range = rospy.get_param("~dt_polyhedron_closest_node_range", 5.0)
    dt_polyhedron_eps = rospy.get_param("~dt_polyhedron_eps", 1.0)
    use_filter_forward_turn_route = rospy.get_param("~use_filter_forward_turn_route", True)
    use_sample_free_points = rospy.get_param("~use_sample_free_points", True)
    sample_free_points_min_distance = rospy.get_param("~sample_free_points_min_distance", 2.0)
    sample_free_points_by_angle_range = rospy.get_param("~sample_free_points_by_angle_range", 12.0)
    sample_free_points_by_angle_unknown_area_margin = rospy.get_param("~sample_free_points_by_angle_unknown_area_margin", 0.5)
    sample_free_points_by_angle_lethal_area_margin = rospy.get_param("~sample_free_points_by_angle_lethal_area_margin", 1.0)
    use_cluster_point_outside_polyhedron_free_area = rospy.get_param("~use_cluster_point_outside_polyhedron_free_area", True)
    costmap_topic = rospy.get_param("~costmap_topic", "/map")
    scan_topic = rospy.get_param("~scan_topic", "/cabot_explore/scan")
    path_topic = rospy.get_param("~path_topic", "/cabot_explore/path")
    lethal_cost_threshold = rospy.get_param("~lethal_cost_threshold", 65)
    costmap_inflation_radius = rospy.get_param("~costmap_inflation_radius", 0.3)
    costmap_resolution = rospy.get_param("~costmap_resolution", 0.20)
    min_frontier_size = rospy.get_param("~min_frontier_size", 2.0)
    use_pause_possible_turn = rospy.get_param("~use_pause_possible_turn", True)
    possible_turn_perpendicular_yaw_tolerance = rospy.get_param("~possible_turn_perpendicular_yaw_tolerance", math.pi/18.0)
    use_frontier_proximity = rospy.get_param("~use_frontier_proximity", True)
    frontier_proximity_threshold = rospy.get_param("~frontier_proximity_threshold", 3.0)
    min_distance_goal_inside_coverage_polyhedron = rospy.get_param("~min_distance_goal_inside_coverage_polyhedron", 1.0)
    max_abs_yaw_forward_goal = rospy.get_param("~max_abs_yaw_forward_goal", math.pi/6.0)
    max_landmark_interval = rospy.get_param("~max_landmark_interval", 5.0)
    landmark_loop_detect_distance = rospy.get_param("~landmark_loop_detect_distance", 10.0)

    explore_manager_node = ExploreManagerNode(use_scan_topic_free_area, use_dt_polyhedron_sensor_range, default_polyhedron_sensor_range, dt_polyhedron_closest_node_range,
                                            dt_polyhedron_eps, use_filter_forward_turn_route, use_sample_free_points, sample_free_points_min_distance, sample_free_points_by_angle_range,
                                            sample_free_points_by_angle_unknown_area_margin, sample_free_points_by_angle_lethal_area_margin, use_cluster_point_outside_polyhedron_free_area, costmap_topic, scan_topic,
                                            path_topic, lethal_cost_threshold, costmap_inflation_radius, costmap_resolution, min_frontier_size, use_pause_possible_turn, possible_turn_perpendicular_yaw_tolerance,
                                            use_frontier_proximity, frontier_proximity_threshold, min_distance_goal_inside_coverage_polyhedron, max_abs_yaw_forward_goal, max_landmark_interval, landmark_loop_detect_distance)

    bagfile = rospy.get_param("~bagfile", "")
    should_augment = rospy.get_param("~augment",False)
    annotationfile = rospy.get_param("~annotationfile","")
    debug = rospy.get_param("~debug", False)

    vis_publisher = rospy.Publisher('/cabot_explore/vis_explore_evaluation', MarkerArray, queue_size=1)

    sub = rospy.Subscriber("/cabot/event", std_msgs.msg.String, event_callback, None)
    mapPub = rospy.Publisher("/map", nav_msgs.msg.OccupancyGrid, queue_size=1, latch=True)
    br = tf.TransformBroadcaster()
    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    bagfile = augmentbag(bagfile, costmap_topic, scan_topic, should_augment)

    print(bagfile)
    print(annotationfile)

    with open(annotationfile) as file:
        annotation = yaml.safe_load(file)

    if bagfile and annotationfile:
        bag = rosbag.Bag(bagfile)

        mapmsg = None
        scanmsg = None
        worldposemsg = None
        odommsg = None

        eval_dict = {"TP":0,"TN":0,"FP":0,"FN":0}

        data = []
        for topic, msg, t in bag.read_messages(topics=[costmap_topic, scan_topic, '/odom', '/world_pose']):
            if topic == costmap_topic:
                mapmsg = msg
            elif topic == scan_topic:
                scanmsg = msg
            elif topic == '/odom':
                odommsg = msg
            elif topic == '/world_pose':
                worldposemsg = msg

            if mapmsg != None and scanmsg != None and odommsg != None and worldposemsg != None:
                data.append(msgs(mapmsg,scanmsg,odommsg,worldposemsg))
                mapmsg = None
                scanmsg = None
                odommsg = None
                worldposemsg = None

        while 0 <= current_idx and current_idx < len(data):
            should_record = not current_idx in recorded_time
            mapmsg = data[current_idx].mapmsg
            scanmsg = data[current_idx].scanmsg
            worldposemsg = data[current_idx].worldposemsg

            p = worldposemsg.position
            q = worldposemsg.orientation
            time = rospy.Time.now()
            br.sendTransform((p.x, p.y, p.z),
                                (q.x, q.y, q.z, q.w),
                                time,
                                "base_footprint",
                                "map")
            mapPub.publish(mapmsg)

            velodyne_to_map_transform_stamped = tf2_buffer.lookup_transform("map", "velodyne", time, rospy.Duration(1.0))
            goal_candidate_poses = explore_manager_node.get_goal_candidates(worldposemsg, mapmsg, scanmsg, velodyne_to_map_transform_stamped.transform)

            in_intersection, idx = isCabotInIntersection(worldposemsg,annotation)
            if in_intersection:
                if isPredictionCorrect(goal_candidate_poses,annotation["intersections"][idx]):
                    result = "TP"
                    print("TP: successfully detected interecion")
                else:
                    result = "FN"
                    explain_why_fail(result, worldposemsg, goal_candidate_poses, max_abs_yaw_forward_goal, len(annotation["intersections"][idx]["ways"]))
                visualize_goals(vis_publisher, worldposemsg, goal_candidate_poses, annotation["intersections"][idx])
            else:
                if candidateStraight(goal_candidate_poses):
                    result = "TN"
                    print("TN: sucessfully detected corridor")
                else:  
                    result = "FP"
                    explain_why_fail(result, worldposemsg, goal_candidate_poses, max_abs_yaw_forward_goal, "2")
                visualize_goals(vis_publisher, worldposemsg, goal_candidate_poses, None)
            next_line = False
            if should_record: eval_dict[result] += 1    
            recorded_time.add(current_idx)

            allow_input = True
            while debug and not next_line:
                sleep(0.001)
            
            if not debug:
                current_idx += 1

        print(eval_dict)
        accuracy = (eval_dict["TP"] + eval_dict["TN"]) / (eval_dict["TP"] + eval_dict["TN"] + eval_dict["FP"] + eval_dict["FN"])
        precision = eval_dict["TP"] / (eval_dict["TP"] + eval_dict["FP"])
        recall = eval_dict["TP"] / (eval_dict["TP"] + eval_dict["FN"])
        f_measure = 2 * precision * recall / (precision + recall)
        print("accuracy:  ",accuracy)
        print("precision: ",precision)
        print("recall:    ",recall)
        print("f_measure: ",f_measure)

def isCabotInIntersection(pose,annotation):
    threshold = 0#[m]
    min_distance = 9999 
    min_idx = 0
    for idx, intersection_data in enumerate(annotation["intersections"]):
        distance = math.sqrt((pose.position.x - intersection_data["position"]["x"])**2+(pose.position.y - intersection_data["position"]["y"])**2)
        if distance<min_distance:
            min_distance = distance
            min_idx = idx

    intersection_center = annotation["intersections"][min_idx]["position"]
    for arrow in annotation["intersections"][min_idx]["ways"]:
        new_threshold = math.sqrt((intersection_center["x"] - arrow["pose"]["position"]["x"])**2+(intersection_center["y"] -arrow["pose"]["position"]["y"])**2)
        if new_threshold > threshold:
            threshold = new_threshold
    return min_distance<threshold, min_idx

def candidateStraight(goal_candidate_poses):
    goal_candidate_poses_filtered = list(filter(None, goal_candidate_poses)) 
    if len(goal_candidate_poses_filtered) != 2: return False

    orientation = goal_candidate_poses_filtered[0].orientation
    w0 = [orientation.x, orientation.y, orientation.z, orientation.w]
    orientation = goal_candidate_poses_filtered[1].orientation
    w1 = [orientation.x, orientation.y, orientation.z, orientation.w]
    angle = diff_abs_yaw(w1,w0)
    return angle > straight_corridor_threshold

def isPredictionCorrect(goal_candidate_poses,intersection):
    goal_candidate_poses_filtered = list(filter(None, goal_candidate_poses)) 
    if len(goal_candidate_poses_filtered) != len(intersection["ways"]): 
        return False
    ways = copy.copy(intersection["ways"])
    for i in reversed(range(len(goal_candidate_poses_filtered))):
        for j in reversed(range(len(ways))):
            orientation = goal_candidate_poses_filtered[i].orientation
            detected_way = [orientation.x, orientation.y, orientation.z, orientation.w]
            way = [ways[j]["pose"]["orientation"]["x"],ways[j]["pose"]["orientation"]["y"],ways[j]["pose"]["orientation"]["z"],ways[j]["pose"]["orientation"]["w"]]
            angle = diff_abs_yaw(detected_way,way)
            if angle < angle_threshold:
                del goal_candidate_poses_filtered[i]
                del ways[j]
                break
    return len(goal_candidate_poses_filtered) == 0 and len(ways) == 0 

def explain_why_fail(result, robot_pose, goal_candidate_poses, max_abs_yaw_forward_goal, len_ways=0):
    has_forward_goal = False
    has_backward_goal = False
    has_left_goal = False
    has_right_goal = False
    min_abs_yaw_backward_goal = math.pi-max_abs_yaw_forward_goal
    for goal_pose in goal_candidate_poses:
        robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
        pose_to_goal_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, goal_pose.position)
        pose_to_goal_quat_diff = geometry_utils.calc_relative_orientation_quaternion(robot_pose_quat, pose_to_goal_quat)
        pose_to_goal_yaw_diff = tf.transformations.euler_from_quaternion(pose_to_goal_quat_diff)[2]
        pose_to_goal_abs_yaw_diff = abs(pose_to_goal_yaw_diff)
        if pose_to_goal_abs_yaw_diff<max_abs_yaw_forward_goal:
            has_forward_goal = True
        elif pose_to_goal_abs_yaw_diff>min_abs_yaw_backward_goal:
            has_backward_goal = True
        elif pose_to_goal_yaw_diff>0:
            has_left_goal = True
        else:
            has_right_goal = True

    can_go_ways_str = []
    if has_forward_goal: can_go_ways_str.append("F")
    if has_backward_goal: can_go_ways_str.append("B")
    if has_left_goal: can_go_ways_str.append("L")
    if has_right_goal: can_go_ways_str.append("R")
    print(result + ": The detected approximate direction was",can_go_ways_str," while the number of ways in the intersection were",len_ways)

def event_callback(msg):
    global current_idx
    global next_line
    global allow_input

    if allow_input:
        if msg.data.startswith("button_down_10"):
            current_idx += 1
            next_line = True
            allow_input = False
        elif msg.data.startswith("button_down_11"):
            current_idx -= 1
            next_line = True
            allow_input = False

def diff_abs_yaw(quat1, quat2):
    quat_diff = geometry_utils.calc_relative_orientation_quaternion(quat1, quat2)
    _, _, yaw_diff = euler_from_quaternion(quat_diff)
    return abs(yaw_diff)

def complement_pose(p1: Pose, p2: Pose) -> Pose:
    import geometry_msgs.msg._Quaternion
    complemented: Pose  = Pose()
    complemented.position.x = (p1.position.x + p2.position.x) * 0.5
    complemented.position.y = (p1.position.y + p2.position.y) * 0.5
    complemented.position.z = (p1.position.z + p2.position.z) * 0.5

    o1 = p1.orientation
    o2 = p2.orientation
    theta = diff_abs_yaw([o1.x,o1.y,o1.z,o1.w], [o2.x,o2.y,o2.z,o2.w])

    t = 0.5
    sin = math.sin(theta)
    c1 = math.sin(theta * t) / sin
    c2 = math.sin(theta * (1.0 - t)) / sin
    
    complemented.orientation.x = c1 * o1.x + c2 * o2.x
    complemented.orientation.y = c1 * o1.y + c2 * o2.y
    complemented.orientation.z = c1 * o1.z + c2 * o2.z
    complemented.orientation.w = c1 * o1.w + c2 * o2.w
    norm = math.sqrt(complemented.orientation.x ** 2 + complemented.orientation.y ** 2 + complemented.orientation.z ** 2 + complemented.orientation.w ** 2 )
    complemented.orientation.x /= norm
    complemented.orientation.y /= norm
    complemented.orientation.z /= norm
    complemented.orientation.w /= norm

    return complemented

def augmentbag(bagfile, costmap_topic, scan_topic, use_augmented_data):
    if not use_augmented_data:
        return bagfile

    with rosbag.Bag(bagfile,'r') as bag:
        with rosbag.Bag('augmented-' + bagfile,'w') as outbag:
            _map = None
            _scan = None
            _odom = None
            _world_pose = None
            _time = None

            _prev_map = None
            _prev_scan = None
            _prev_odom = None
            _prev_world_pose = None
            _prev_time = None

            for topic, msg, time in bag.read_messages(topics=[costmap_topic, scan_topic, '/odom', '/world_pose']):
                if topic == costmap_topic:
                    _map = msg
                elif topic == scan_topic:
                    _scan = msg
                elif topic == "/odom":
                    _odom = msg
                elif topic == "/world_pose":
                    _world_pose = msg
                _time = time

                if _map != None and _scan != None and _odom != None and _world_pose != None and _time != None:
                    if _prev_map != None and _prev_scan != None and _prev_odom != None and _prev_world_pose != None:
                        _complented_odom = copy.deepcopy(_odom)
                        _complented_odom.pose.pose = complement_pose(_odom.pose.pose, _prev_odom.pose.pose)
                        _complemented_world_pose = complement_pose(_world_pose, _prev_world_pose)
                        _complemeted_time = rospy.Time((_time.to_sec() + _prev_time.to_sec())/2)
                        outbag.write(costmap_topic, _map, _complemeted_time)
                        outbag.write(scan_topic, _scan, _complemeted_time)
                        outbag.write("/odom", _complented_odom, _complemeted_time)
                        outbag.write("/world_pose", _complemented_world_pose, _complemeted_time)

                    outbag.write(costmap_topic, _map, _time)
                    outbag.write(scan_topic, _scan, _time)
                    outbag.write("/odom", _odom, _time)
                    outbag.write("/world_pose", _world_pose, _time)

                    _prev_map = _map
                    _prev_scan = _scan
                    _prev_odom = _odom
                    _prev_world_pose = _world_pose
                    _prev_time = _time

                    _map = None
                    _scan = None
                    _odom = None
                    _world_pose = None
                    _time = None
            outbag.close()
        bag.close()
        return 'augmented-' + bagfile

def visualize_goals(vis_publisher, robot_pose, goal_candidate_poses, intersection):
    arrow_length = 0.5
    goal_candidate_direction_margin = 1.0
    red_rgb = matplotlib.colors.to_rgb('red')
    green_rgb = matplotlib.colors.to_rgb('green')
    blue_rgb = matplotlib.colors.to_rgb('blue')

    marker_array = MarkerArray()
    marker = Marker()
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)

    for goal_idx, goal_candidate_pose in enumerate(goal_candidate_poses):
        arrow_start = goal_candidate_pose.position
        goal_candidate_pose_quat = (goal_candidate_pose.orientation.x, goal_candidate_pose.orientation.y, goal_candidate_pose.orientation.z, goal_candidate_pose.orientation.w)
        goal_candidate_pose_yaw = tf.transformations.euler_from_quaternion(goal_candidate_pose_quat)[2]
        arrow_end = Point(arrow_start.x+arrow_length*math.cos(goal_candidate_pose_yaw), arrow_start.y+arrow_length*math.sin(goal_candidate_pose_yaw), 0.0)

        marker = visualize_utils.create_rviz_marker("explore-evaluation-goal-candidates", goal_idx, Marker.ARROW, scale_x=0.1, scale_y=0.2,
                                                    color_r=blue_rgb[0], color_g=blue_rgb[1], color_b=blue_rgb[2])
        marker.points.append(arrow_start)
        marker.points.append(arrow_end)
        marker_array.markers.append(marker)

        goal_candidate_pose_quat = (goal_candidate_pose.orientation.x, goal_candidate_pose.orientation.y, goal_candidate_pose.orientation.z, goal_candidate_pose.orientation.w)
        goal_candidate_pose_yaw = tf.transformations.euler_from_quaternion(goal_candidate_pose_quat)[2]
        arrow_start = Point(robot_pose.position.x + goal_candidate_direction_margin*math.cos(goal_candidate_pose_yaw), \
                            robot_pose.position.y + goal_candidate_direction_margin*math.sin(goal_candidate_pose_yaw), 0.0)
        arrow_end = Point(arrow_start.x+arrow_length*math.cos(goal_candidate_pose_yaw), arrow_start.y+arrow_length*math.sin(goal_candidate_pose_yaw), 0.0)

        marker = visualize_utils.create_rviz_marker("explore-evaluation-goal-candidates-direction", goal_idx, Marker.ARROW, scale_x=0.1, scale_y=0.2,
                                                    color_r=green_rgb[0], color_g=green_rgb[1], color_b=green_rgb[2])
        marker.points.append(arrow_start)
        marker.points.append(arrow_end)
        marker_array.markers.append(marker)

    if intersection is not None:
        intersection_point = Point(intersection["position"]["x"], intersection["position"]["y"], 0.0)
        marker = visualize_utils.create_rviz_marker("explore-evaluation-goal-intersection-point", 0, Marker.CYLINDER, position=intersection_point,
                                                    scale_x=0.1, scale_y=0.1, scale_z=0.1, color_r=red_rgb[0], color_g=red_rgb[1], color_b=red_rgb[2])
        marker_array.markers.append(marker)

        for way_idx, way in enumerate(intersection["ways"]):
            way_quat = (way["pose"]["orientation"]["x"], way["pose"]["orientation"]["y"], way["pose"]["orientation"]["z"], way["pose"]["orientation"]["w"])
            way_yaw = tf.transformations.euler_from_quaternion(way_quat)[2]

            arrow_start = intersection_point
            arrow_end = Point(arrow_start.x+arrow_length*math.cos(way_yaw), arrow_start.y+arrow_length*math.sin(way_yaw), 0.0)

            marker = visualize_utils.create_rviz_marker("explore-evaluation-goal-intersection-point", way_idx, Marker.ARROW, scale_x=0.1, scale_y=0.2,
                                                        color_r=red_rgb[0], color_g=red_rgb[1], color_b=red_rgb[2])
            marker.points.append(arrow_start)
            marker.points.append(arrow_end)
            marker_array.markers.append(marker)

    vis_publisher.publish(marker_array)

if __name__ == "__main__":
    main()
