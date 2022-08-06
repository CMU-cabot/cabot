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

#include <tf2/LinearMath/Quaternion.h>

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/flann/flann.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "cabot_navigation2/cabot_planner_util.hpp"
#include "cabot_navigation2/navcog_path_util.hpp"

namespace cabot_navigation2 {

enum DetourMode { LEFT, RIGHT, IGNORE };

struct CaBotPlannerOptions {
  float initial_node_interval_meter = 0.20;
  float devide_link_cell_interval_threshold = 8;  // = 2 * initial_node_interval_meter / resolution
  float optimize_distance_from_start = 10.0;
  float iteration_scale = 0.01;
  float iteration_scale_interval = -0.01;
  float gravity_factor = 1.0;
  float link_spring_factor = 2.0;
  float link_straighten_factor = 2.0;
  float anchor_spring_factor = 0.01;
  float complete_threshold = 0.005;
  float obstacle_margin = 2.0;
  float min_distance_to_obstacle = 1.00;
  float min_distance_to_obstacle_group = 0.1;
  float min_anchor_length = 0.1;
  float min_link_length = 0.01;
  float go_around_detect_threshold = M_PI * 4;
  int cost_lethal_threshold = 253;
  int cost_pass_threshold = 253;
  int max_obstacle_scan_distance = 100;
  int interim_plan_publish_interval = 10;
  int kdtree_search_radius_in_cells = 100;
  int kdtree_max_results = 100;
  int min_iteration_count = 10;
  int max_iteration_count = 500;
  bool fix_node = false;
  bool adjust_start = false;
  bool use_navcog_path_on_failure = false;
};

class CaBotPlannerParam;

class CaBotPlan {
 public:
  CaBotPlan(CaBotPlannerParam &param_);
  CaBotPlannerParam &param;
  std::vector<Node> nodes;
  std::vector<Node> nodes_backup;
  unsigned long start_index;
  unsigned long end_index;
  DetourMode detour_mode;
  bool okay;

  void resetNodes();
  void findIndex();
  void adjust();
  float length();
  std::vector<Node> getTargetNodes();
  nav_msgs::msg::Path getPlan(bool normalized, float normalize_length = 0.02);
};

class CaBotPlannerParam {
  // input
 public:
  CaBotPlannerParam(CaBotPlannerOptions &options_, const geometry_msgs::msg::PoseStamped &start_,
                    const geometry_msgs::msg::PoseStamped &goal_, nav_msgs::msg::Path navcog_path_,
                    people_msgs::msg::People::SharedPtr people_msg_ptr_,
                    people_msgs::msg::People::SharedPtr obstacles_msg_ptr_, nav2_costmap_2d::Costmap2D *costmap,
                    nav2_costmap_2d::Costmap2D *static_costmap);

  ~CaBotPlannerParam();

  CaBotPlannerOptions &options;
  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  nav_msgs::msg::Path navcog_path;
  people_msgs::msg::People people_msg;
  people_msgs::msg::People obstacles_msg;
  nav2_costmap_2d::Costmap2D *costmap;
  nav2_costmap_2d::Costmap2D *static_costmap;

  void allocate();
  void deallocate();
  bool adjustPath();
  void setCost();
  void clearCostAround(people_msgs::msg::Person &person);
  void scanObstacleAt(ObstacleGroup &group, float mx, float my, unsigned int min_obstacle_cost, float max_dist);
  void findObstacles(std::vector<Node> nodes);

  bool worldToMap(float wx, float wy, float &mx, float &my) const;
  void mapToWorld(float mx, float my, float &wx, float &wy) const;
  int getIndex(float x, float y) const;
  int getIndexByPoint(Point &p) const;
  std::vector<Node> getNodes() const;
  std::vector<Obstacle> getObstaclesNearPoint(const Point &node, bool collision) const;

  // internal
  rclcpp::Logger logger{rclcpp::get_logger("CaBotPlannerParam")};
  int width;
  int height;
  float origin_x;
  float origin_y;
  float resolution;
  nav_msgs::msg::Path path;
  unsigned char *cost;
  unsigned char *static_cost;
  unsigned short *mark;
  // void setParam(int width, int height, float origin_x, float origin_y, float resolution, DetourMode detour);

  std::vector<Obstacle> obstacles;
  std::vector<ObstacleGroup> groups;
  std::vector<Obstacle> olist_;
  std::vector<Obstacle> olist_non_collision_;
  cv::Mat *data_;
  cv::Mat *data_non_collision_;
  cv::flann::Index *idx_;
  cv::flann::Index *idx_non_collision_;
};

}  // namespace cabot_navigation2