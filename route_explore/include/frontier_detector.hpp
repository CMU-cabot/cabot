// Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __ROUTE_EXPLORE_FRONTIER_DETECTOR_HPP__
#define __ROUTE_EXPLORE_FRONTIER_DETECTOR_HPP__

#include <math.h>
#include <ros/ros.h>

namespace RouteExplore {

    struct Frontier {
        geometry_msgs::Point centroid;
        std::vector<geometry_msgs::Point> points;
    };

    float calcPointDistance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
        return std::sqrt(std::pow(point1.x-point2.x, 2.0) + std::pow(point1.y-point2.y, 2.0));
    }

    geometry_msgs::Point calcCentroidPointVec(const std::vector<geometry_msgs::Point>& point_vec) {
        geometry_msgs::Point centroid_point;
        centroid_point.x = 0.0;
        centroid_point.y = 0.0;
        centroid_point.z = 0.0;
        for (const auto& point : point_vec) {
            centroid_point.x += point.x;
            centroid_point.y += point.y;
        }
        centroid_point.x /= float(point_vec.size());
        centroid_point.y /= float(point_vec.size());
        return centroid_point;
    }

    std::vector<int> get4Neighbor(int costmap_width, int costmap_height, int costmap_x, int costmap_y) {
        std::vector<int> neighbor_points;
        if (costmap_x>0) {
            neighbor_points.push_back(costmap_y*costmap_width + costmap_x-1);
        }
        if (costmap_y>0) {
            neighbor_points.push_back((costmap_y-1)*costmap_width + costmap_x);
        }
        if (costmap_x<costmap_width-1) {
            neighbor_points.push_back(costmap_y*costmap_width + costmap_x+1);
        }
        if (costmap_y<costmap_height-1) {
            neighbor_points.push_back((costmap_y+1)*costmap_width + costmap_x);
        }
        return neighbor_points;
    }

    std::vector<int> get8Neighbor(int costmap_width, int costmap_height, int costmap_x, int costmap_y) {
        std::vector<int> neighbor_points = get4Neighbor(costmap_width, costmap_height, costmap_x, costmap_y);

        if (costmap_x>0 && costmap_y>0) {
            neighbor_points.push_back((costmap_y-1)*costmap_width + costmap_x-1);
        }
        if (costmap_x<costmap_width-1 && costmap_y>0) {
            neighbor_points.push_back((costmap_y-1)*costmap_width + costmap_x+1);
        }
        if (costmap_x<costmap_width-1 && costmap_y<costmap_height-1) {
            neighbor_points.push_back((costmap_y+1)*costmap_width + costmap_x+1);
        }
        if (costmap_x>0 && costmap_y<costmap_height-1) {
            neighbor_points.push_back((costmap_y+1)*costmap_width + costmap_x-1);
        }
        return neighbor_points;
    }

    bool has4NeighborUnknown(const nav_msgs::OccupancyGrid& msg, int costmap_x, int costmap_y) {
        std::vector<int> neighbor_points = get4Neighbor(msg.info.width, msg.info.height, costmap_x, costmap_y);
        for (const auto& neighbor_point : neighbor_points) {
            if (msg.data.at(neighbor_point)<0) {
                return true;
            }
        }
        return false;
    }

    bool has8NeighborUnknown(const nav_msgs::OccupancyGrid& msg, int costmap_x, int costmap_y) {
        std::vector<int> neighbor_points = get8Neighbor(msg.info.width, msg.info.height, costmap_x, costmap_y);
        for (const auto& neighbor_point : neighbor_points) {
            if (msg.data.at(neighbor_point)<0) {
                return true;
            }
        }
        return false;
    }

    void mapToWorld(const nav_msgs::OccupancyGrid& msg, int costmap_x, int costmap_y, float& world_x, float& world_y) {
        world_x = msg.info.origin.position.x + (costmap_x + 0.5) * msg.info.resolution;
        world_y = msg.info.origin.position.y + (costmap_y + 0.5) * msg.info.resolution;
    }

    void worldToMap(const nav_msgs::OccupancyGrid& msg, float world_x, float world_y, int& costmap_x, int& costmap_y) {
        costmap_x = int((world_x - msg.info.origin.position.x) / msg.info.resolution);
        costmap_y = int((world_y - msg.info.origin.position.y) / msg.info.resolution);
    }

    // Extract frontier for input point by Wavefront Frontier Detector (WFD)
    // See Algorithms 3.2 in Keidar et. al. IJRR 2014 (https://journals.sagepub.com/doi/abs/10.1177/0278364913494911?journalCode=ijra)
    Frontier extractFrontier(const nav_msgs::OccupancyGrid& msg, int lethal_cost_threshold, std::set<int>& map_close_set, int p, std::set<int>& map_frontier_points) {
        // start Algorithms 3.2
        std::set<int> frontier_open_set;
        std::set<int> frontier_close_set;

        std::deque<int> queue;
        Frontier frontier;

        queue.push_back(p);
        frontier_open_set.insert(p);

        while (!queue.empty()) {
            int q = queue.front();
            queue.pop_front();
            if (map_close_set.find(q)!=map_close_set.end() || frontier_close_set.find(q)!=frontier_close_set.end()) {
                continue;
            }

            int q_x = q % msg.info.width;
            int q_y = q / msg.info.width;
            if (has8NeighborUnknown(msg, q_x, q_y)) {
                float world_q_x;
                float world_q_y;
                mapToWorld(msg, q_x, q_y, world_q_x, world_q_y);

                geometry_msgs::Point q_point;
                q_point.x = world_q_x;
                q_point.y = world_q_y;
                q_point.z = 0.0;
                frontier.points.push_back(q_point);
                map_frontier_points.insert(q);

                std::vector<int> neighbor_points = get8Neighbor(msg.info.width, msg.info.height, q_x, q_y);
                for (const auto& neighbor_point : neighbor_points) {
                    int neighbor_point_cost = msg.data.at(neighbor_point);
                    if (neighbor_point_cost>=0 && neighbor_point_cost<lethal_cost_threshold && frontier_open_set.find(neighbor_point)==frontier_open_set.end()
                            && frontier_close_set.find(neighbor_point)==frontier_close_set.end() && map_close_set.find(neighbor_point)==map_close_set.end()) {
                        queue.push_back(neighbor_point);
                        frontier_open_set.insert(neighbor_point);
                    }
                }
            }
            frontier_close_set.insert(q);
        }

        // calculate centroid of extracted frontier (select the point which is closest to centroid of points to guarantee reachability)
        geometry_msgs::Point centroid_frontier_points = calcCentroidPointVec(frontier.points);

        geometry_msgs::Point centroid_closest_frontier_point;
        float centroid_closest_frontier_point_distance = std::numeric_limits<float>::max();
        for (const auto& point : frontier.points) {
            float distance = calcPointDistance(point, centroid_frontier_points);
            if (distance<centroid_closest_frontier_point_distance) {
                centroid_closest_frontier_point = point;
                centroid_closest_frontier_point_distance = distance;
            }
        }

        frontier.centroid = centroid_closest_frontier_point;
        return frontier;
    }

    // Detect frontier points by Wavefront Frontier Detector (WFD)
    // See Algorithms 3.1 in Keidar et. al. IJRR 2014 (https://journals.sagepub.com/doi/abs/10.1177/0278364913494911?journalCode=ijra)
    std::vector<Frontier> detectFrontierPointsWfd(const nav_msgs::OccupancyGrid& msg, int lethal_cost_threshold, const tf2::Transform& robot_pose, float sensor_range=-1.0) {
        int robot_pose_costmap_x;
        int robot_pose_costmap_y;
        worldToMap(msg, robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), robot_pose_costmap_x, robot_pose_costmap_y);

        float sensor_range_costmap = -1.0;
        if (sensor_range>0) {
            // prepare search range for limiting search area
            sensor_range_costmap = sensor_range/msg.info.resolution;
        }

        // start Algorithms 3.1
        std::set<int> map_open_set;
        std::set<int> map_close_set;

        std::deque<int> queue;
        int robot_pose_costmap_idx = robot_pose_costmap_y*msg.info.width + robot_pose_costmap_x;
        queue.push_back(robot_pose_costmap_idx);
        map_open_set.insert(robot_pose_costmap_idx);

        std::vector<Frontier> frontier_list;
        while (!queue.empty()) {
            int p = queue.front();
            queue.pop_front();
            if (map_close_set.find(p)!=map_close_set.end()) {
                continue;
            }

            int p_x = p % msg.info.width;
            int p_y = p / msg.info.width;
            if (has4NeighborUnknown(msg, p_x, p_y)) {
                std::set<int> map_frontier_points;
                Frontier frontier = extractFrontier(msg, lethal_cost_threshold, map_close_set, p, map_frontier_points);
                frontier_list.push_back(frontier);
                map_close_set.insert(map_frontier_points.begin(), map_frontier_points.end());
            }

            std::vector<int> p_neighbor_points = get4Neighbor(msg.info.width, msg.info.height, p_x, p_y);
            for (const auto& v : p_neighbor_points) {
                int v_x = v % msg.info.width;
                int v_y = v / msg.info.width;

                if (sensor_range_costmap > 0) {
                    float distance = std::sqrt(std::pow(v_x-robot_pose_costmap_x, 2.0) + std::pow(v_y-robot_pose_costmap_y, 2.0));
                    if (distance>sensor_range_costmap) {
                        continue;
                    }
                }

                int v_cost = msg.data.at(v);
                if (v_cost>=0 && v_cost<lethal_cost_threshold && map_open_set.find(v)==map_open_set.end() && map_close_set.find(v)==map_close_set.end()) {
                    std::vector<int> v_neighbor_points = get4Neighbor(msg.info.width, msg.info.height, v_x, v_y);
                    bool has_map_open_space_neighbor = false;
                    for (const auto& v_neighbor_point : v_neighbor_points) {
                        int v_neighbor_point_cost = msg.data.at(v_neighbor_point);
                        if (v_neighbor_point_cost>=0 && v_neighbor_point_cost<lethal_cost_threshold) {
                            has_map_open_space_neighbor = true;
                            break;
                        }
                    }
                    if (has_map_open_space_neighbor) {
                        queue.push_back(v);
                        map_open_set.insert(v);
                    }
                }
            }
            map_close_set.insert(p);
        }
        return frontier_list;
    }

}

#endif //__ROUTE_EXPLORE_FRONTIER_DETECTOR_HPP__