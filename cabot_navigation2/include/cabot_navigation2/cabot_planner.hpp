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

#ifndef CABOT_NAVIGATION2__CABOT_PLANNER_HPP_
#define CABOT_NAVIGATION2__CABOT_PLANNER_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <people_msgs/msg/people.hpp>
#include <queue_msgs/msg/queue.hpp>
#include <opencv2/flann/flann.hpp>
#include "cabot_navigation2/cabot_planner_param.hpp"

namespace cabot_navigation2
{

class CaBotPlanner : public nav2_core::GlobalPlanner
{
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
  nav_msgs::msg::Path createPlan(CaBotPlannerParam & param);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void peopleCallback(const people_msgs::msg::People::SharedPtr peopole);
  void obstaclesCallback(const people_msgs::msg::People::SharedPtr obstacles);
  void queueCallback(const queue_msgs::msg::Queue::SharedPtr queue);

protected:
  float iterate(const CaBotPlannerParam & param, CaBotPlan & plan, int count);
  void debug_output(const CaBotPlannerParam & param, CaBotPlan & plan);
  bool checkPath(const CaBotPlannerParam & param, CaBotPlan & plan);
  bool checkGoAround(const CaBotPlannerParam & param, CaBotPlan & plan);

private:
  rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params);
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("CaBotPlanner")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string static_layer_name_;
  std::string inflation_layer_name_;
  CostmapLayerCapture * costmap_capture_;
  CostmapLayerCapture * static_costmap_capture_;

  std::recursive_mutex mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  CaBotPlannerOptions options_;
  PathEstimateOptions pe_options_;
  nav_msgs::msg::Path::SharedPtr navcog_path_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  people_msgs::msg::People::SharedPtr last_people_;
  people_msgs::msg::People::SharedPtr last_obstacles_;
  queue_msgs::msg::Queue::SharedPtr last_queue_;

  std::string path_topic_;
  int cost_threshold_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

  bool path_debug_;
  std::chrono::system_clock::time_point last_iteration_path_published_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr iteration_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr right_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr left_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr obstacle_points_pub_;
  std::string iteration_path_topic_;
  std::string right_path_topic_;
  std::string left_path_topic_;
  std::string obstacle_points_topic_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_costmap_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<people_msgs::msg::People>::SharedPtr target_people_pub_;
  rclcpp_lifecycle::LifecyclePublisher<people_msgs::msg::People>::SharedPtr target_obstacles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  std::string static_costmap_topic_;
  std::string costmap_topic_;
  std::string target_path_topic_;
  std::string target_people_topic_;
  std::string target_obstacles_topic_;
  std::string target_goal_pose_topic_;
  std::string current_pose_pub_topic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<queue_msgs::msg::Queue>::SharedPtr queue_sub_;
  std::string odom_topic_;
  std::string people_topic_;
  std::string obstacles_topic_;
  std::string queue_topic_;
};

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__CABOT_PLANNER_HPP_
