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
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/flann/flann.hpp>
#include "cabot_navigation2/cabot_planner_util.hpp"
#include "cabot_navigation2/navcog_path_util.hpp"

namespace cabot_navigation2 {

enum DetourMode {
  LEFT,
  RIGHT,
  IGNORE
};

struct CaBotPlannerOptions {
  float optimize_distance_from_start = 10.0;
  float iteration_scale = 0.25;
  float iteration_scale_interval = 0.001;
  float gravity_factor = 1.0;
  float link_spring_factor = 1.0;
  float anchor_spring_factor = 0.01;
  float complete_threshold = 0.001;
  float obstacle_margin = 2;
  float min_distance_to_obstacle = 0.5;
  float min_anchor_length = 0.1;
  float min_link_length = 0.01;
  float go_around_detect_threshold = M_PI * 7 / 4;
  int cost_lethal_threshold = 253;
  int cost_pass_threshold = 128;
  int max_obstacle_scan_distance = 50;
  int interim_plan_publish_interval = 100;
  int kdtree_search_radius_in_cells = 100;
  int kdtree_max_results = 100;
};

class CaBotPlanner : public nav2_core::GlobalPlanner {
 public:
  CaBotPlanner();
  ~CaBotPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  void pathCallBack(const nav_msgs::msg::Path::SharedPtr path);

  // for debug visualization
  nav_msgs::msg::Path getPlan(bool normalized=false, float normalize_length=0.1);
    
 protected:
  void setParam(int width, int height, float origin_x, float origin_y, float resolution, DetourMode detour);
  bool worldToMap(float wx, float wy, float & mx, float & my);
  void mapToWorld(float mx, float my, float & wx, float & wy);
  int getIndex(float x, float y);
  int getIndexByPoint(Point & p);
  void setCost(unsigned char* cost);
  void setPath(nav_msgs::msg::Path path);
  bool iterate();
  int iterate_counter_;
 
  void resetNodes();
  std::vector<Node> getNodesFromPath(nav_msgs::msg::Path path);
  void findObstacles();
  void scanObstacleAt(ObstacleGroup & group, float mx, float my, unsigned int cost, float max_dist=100);
  std::vector<Obstacle> getObstaclesNearNode(Node & node);
 private:
  rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params);
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("CaBotPlanner")};
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  CaBotPlannerOptions options_;
  nav_msgs::msg::Path::SharedPtr navcog_path_;
  std::string path_topic_;
  int cost_threshold_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

  bool path_debug_;
  std::chrono::system_clock::time_point last_iteration_path_published_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr iteration_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr right_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr left_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr obstacles_pub_;
  std::string iteration_path_topic_;
  std::string right_path_topic_;
  std::string left_path_topic_;
  std::string obstacles_topic_;

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
  std::set<Obstacle> obstacles_;
  std::set<ObstacleGroup> groups_;
  std::vector<Obstacle> olist_;
  cv::Mat *data_;
  cv::flann::Index *idx_;
};

}

