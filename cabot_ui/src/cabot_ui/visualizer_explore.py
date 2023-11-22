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

import math

from geometry_msgs.msg import Point
import matplotlib
import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray

from route_explore_utils import visualize_utils


class VisualizerExplore(object):
    def __init__(self):
        self.reset()
        self.color_list = [matplotlib.colors.to_rgb(cname) for cname in matplotlib.colors.TABLEAU_COLORS.keys()]

        self.global_map_name = "map"

        self.detected_turns_pub = rospy.Publisher("/cabot_explore/detected_turns", MarkerArray, queue_size=10, latch=True)


    def reset(self):
        self.coverage_polyhedron = None
        self.pose_to_turn = None
        self.goal_candidate_poses = None
        self.visible_notified_turn_areas = None
        self.missing_notified_turn_areas = None
        # visualize turn area attributes for test
        self.visible_notified_turn_areas_attributes = None
        self.visible_notified_turn_areas_route_confidence = None


    def visualize(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        # draw coverage polyhedron
        if self.coverage_polyhedron is not None:
            marker_rgb = matplotlib.colors.to_rgb('lightyellow')

            coverage_polyhedron_points = self.coverage_polyhedron.points[self.coverage_polyhedron.vertices]
            for point_idx, point in enumerate(coverage_polyhedron_points):
                marker = visualize_utils.create_rviz_marker("visualize-explore-coverage-polyhedron-point", point_idx, Marker.CYLINDER, position=Point(point[0], point[1], 0.0),
                                                            color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
                marker_array.markers.append(marker)

            for simplex_idx, simplex in enumerate(self.coverage_polyhedron.simplices):
                assert(len(simplex)==2)

                marker = visualize_utils.create_rviz_marker("visualize-explore-coverage-polyhedron-line", simplex_idx, Marker.LINE_STRIP, scale_x=0.05, scale_y=0.05,
                                                            color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
                marker.points.append(Point(self.coverage_polyhedron.points[simplex[0], 0], self.coverage_polyhedron.points[simplex[0], 1], 0.0))
                marker.points.append(Point(self.coverage_polyhedron.points[simplex[1], 0], self.coverage_polyhedron.points[simplex[1], 1], 0.0))
                marker_array.markers.append(marker)

        # draw pose to turn
        if self.pose_to_turn is not None:
            marker_rgb = matplotlib.colors.to_rgb('purple')

            pose_to_turn_quat = (self.pose_to_turn.orientation.x, self.pose_to_turn.orientation.y, self.pose_to_turn.orientation.z, self.pose_to_turn.orientation.w)
            pose_to_turn_yaw = tf.transformations.euler_from_quaternion(pose_to_turn_quat)[2]

            arrow_length = 0.7
            arrow_start = self.pose_to_turn.position
            arrow_end = Point(arrow_start.x+arrow_length*math.cos(pose_to_turn_yaw), arrow_start.y+arrow_length*math.sin(pose_to_turn_yaw), 0.0)

            marker = visualize_utils.create_rviz_marker("visualize-explore-pose-to-turn-goal", 0, Marker.ARROW, scale_x=0.1, scale_y=0.2,
                                                        color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
            marker.points.append(arrow_start)
            marker.points.append(arrow_end)
            marker_array.markers.append(marker)

        # draw goal candidates
        if self.goal_candidate_poses is not None:
            for idx, goal_candidate_pose in enumerate(self.goal_candidate_poses):
                goal_candidate_pose_quat = (goal_candidate_pose.orientation.x, goal_candidate_pose.orientation.y, goal_candidate_pose.orientation.z, goal_candidate_pose.orientation.w)
                goal_candidate_pose_yaw = tf.transformations.euler_from_quaternion(goal_candidate_pose_quat)[2]

                arrow_length = 0.7
                arrow_start = goal_candidate_pose.position
                arrow_end = Point(arrow_start.x+arrow_length*math.cos(goal_candidate_pose_yaw), arrow_start.y+arrow_length*math.sin(goal_candidate_pose_yaw), 0.0)

                color_idx = idx % len(self.color_list)
                marker = visualize_utils.create_rviz_marker("explore-manager-goal-candidate", idx, Marker.ARROW, scale_x=0.1, scale_y=0.2,
                                                            color_r=self.color_list[color_idx][0], color_g=self.color_list[color_idx][1], color_b=self.color_list[color_idx][2])
                marker.points.append(arrow_start)
                marker.points.append(arrow_end)
                marker_array.markers.append(marker)

        # draw visible notified turn areas
        if self.visible_notified_turn_areas is not None:
            for area_idx, visible_notified_turn_area in enumerate(self.visible_notified_turn_areas):
                color_idx = area_idx % len(self.color_list)
                visible_notified_turn_area_contour = visible_notified_turn_area.contour
                for point_idx in range(visible_notified_turn_area_contour.shape[0]):
                    marker = visualize_utils.create_rviz_marker("visualize-explore-visible-notified-turn-areas-"+str(area_idx), point_idx, Marker.LINE_STRIP, scale_x=0.10, scale_y=0.10,
                                                                color_r=self.color_list[color_idx][0], color_g=self.color_list[color_idx][1], color_b=self.color_list[color_idx][2])
                    if point_idx<visible_notified_turn_area_contour.shape[0]-1:
                        marker.points.append(Point(visible_notified_turn_area_contour[point_idx][0], visible_notified_turn_area_contour[point_idx][1], 0.0))
                        marker.points.append(Point(visible_notified_turn_area_contour[point_idx+1][0], visible_notified_turn_area_contour[point_idx+1][1], 0.0))
                    else:
                        marker.points.append(Point(visible_notified_turn_area_contour[point_idx][0], visible_notified_turn_area_contour[point_idx][1], 0.0))
                        marker.points.append(Point(visible_notified_turn_area_contour[0][0], visible_notified_turn_area_contour[0][1], 0.0))
                    marker_array.markers.append(marker)

                # draw turn area attributes for test
                if ("mean" in self.visible_notified_turn_areas_attributes[area_idx]) and ("length" in self.visible_notified_turn_areas_attributes[area_idx]) and ("width" in self.visible_notified_turn_areas_attributes[area_idx]) \
                     and ("area" in self.visible_notified_turn_areas_attributes[area_idx]):
                    area_mean = self.visible_notified_turn_areas_attributes[area_idx]["mean"]
                    area_length = self.visible_notified_turn_areas_attributes[area_idx]["length"]
                    area_width = self.visible_notified_turn_areas_attributes[area_idx]["width"]
                    area_area = self.visible_notified_turn_areas_attributes[area_idx]["area"]
                    area_route_conf = self.visible_notified_turn_areas_route_confidence[area_idx]
                    area_text = "length=" + '{:.1f}'.format(area_length) + ", width=" + '{:.1f}'.format(area_width) + ", area=" + '{:.1f}'.format(area_area) + ", confidence=" + '{:.1f}'.format(area_route_conf)
                    marker = visualize_utils.create_rviz_marker("visualize-explore-visible-notified-turn-areas-text", area_idx, Marker.TEXT_VIEW_FACING, position=Point(area_mean[0], area_mean[1], 1.0), text=str(area_text))
                    marker_array.markers.append(marker)

        # draw missing notified turn areas
        if self.missing_notified_turn_areas is not None:
            for area_idx, missing_notified_turn_area in enumerate(self.missing_notified_turn_areas):
                color_idx = (area_idx + len(self.visible_notified_turn_areas)) % len(self.color_list)
                missing_notified_turn_area_contour = missing_notified_turn_area.contour
                for point_idx in range(missing_notified_turn_area_contour.shape[0]):
                    marker = visualize_utils.create_rviz_marker("visualize-explore-missing-notified-turn-areas-"+str(area_idx), point_idx, Marker.LINE_STRIP, scale_x=0.05, scale_y=0.05,
                                                                color_r=self.color_list[color_idx][0], color_g=self.color_list[color_idx][1], color_b=self.color_list[color_idx][2])
                    if point_idx<missing_notified_turn_area_contour.shape[0]-1:
                        marker.points.append(Point(missing_notified_turn_area_contour[point_idx][0], missing_notified_turn_area_contour[point_idx][1], 0.0))
                        marker.points.append(Point(missing_notified_turn_area_contour[point_idx+1][0], missing_notified_turn_area_contour[point_idx+1][1], 0.0))
                    else:
                        marker.points.append(Point(missing_notified_turn_area_contour[point_idx][0], missing_notified_turn_area_contour[point_idx][1], 0.0))
                        marker.points.append(Point(missing_notified_turn_area_contour[0][0], missing_notified_turn_area_contour[0][1], 0.0))
                    marker_array.markers.append(marker)

        self.detected_turns_pub.publish(marker_array)


instance = VisualizerExplore()
