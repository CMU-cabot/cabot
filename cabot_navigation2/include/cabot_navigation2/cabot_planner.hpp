/*******************************************************************************
 * Copyright (c) 2022  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_navfn_planner/navfn.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/flann/flann.hpp>
#include "cabot_navigation2/cabot_planner_util.hpp"

namespace cabot_planner {

enum DetourMode {
  LEFT,
  RIGHT
};

class Planner {
 public:
  void setParam(int width, int height, float origin_x, float origin_y, float resolution, DetourMode detour);
  bool worldToMap(float wx, float wy, float & mx, float & my);
  void mapToWorld(float mx, float my, float & wx, float & wy);
  int getIndex(float x, float y);
  int getIndexByPoint(Point & p);
  void setCost(unsigned char* cost);
  void setPath(nav_msgs::msg::Path path);
  bool plan(std::chrono::duration<int64_t, std::milli> period);
  void prepare();
  bool iterate();
  nav_msgs::msg::Path getPlan(void);
 protected:
  std::vector<Node> findNodesNearObstacle(Obstacle obstacle, float distance);
  std::vector<Link*> findLinksNearObstacle(Obstacle obstacle, float distance);
  std::vector<Node> getNodesFromPath(nav_msgs::msg::Path path);
  void findObstacles();
  void scanObstacleAt(ObstacleGroup & group, float mx, float my, unsigned int cost);
  std::vector<Link> getLinksFromNodes(std::vector<Node> & nodes);
  std::vector<Obstacle> getObstaclesNearNode(Node & node);
 private:
  int width_;
  int height_;
  float origin_x_;
  float origin_y_;
  float resolution_;
  DetourMode detour_;
  unsigned char* cost_;
  unsigned char* mark_;
  nav_msgs::msg::Path path_;
  std::vector<Node> nodes_;
  std::vector<Link> links_;
  std::set<Obstacle> obstacles_;
  std::set<ObstacleGroup> groups_;
  std::vector<Obstacle> olist_;
  cv::Mat *data_;
  cv::flann::Index *idx_;
};

}

