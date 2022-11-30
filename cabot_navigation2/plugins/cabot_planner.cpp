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

#include "cabot_navigation2/cabot_planner.hpp"

#include <tf2/utils.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <mutex>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotPlanner, nav2_core::GlobalPlanner)

namespace cabot_navigation2 {

CaBotPlanner::CaBotPlanner()
    : costmap_capture_(nullptr),
      static_costmap_capture_(nullptr),
      last_iteration_path_published_(std::chrono::system_clock::now()) {}

CaBotPlanner::~CaBotPlanner() {}

void CaBotPlanner::cleanup() {
  if (costmap_capture_) delete costmap_capture_;
  if (static_costmap_capture_) delete static_costmap_capture_;
}

void CaBotPlanner::activate() {
  iteration_path_pub_->on_activate();
  right_path_pub_->on_activate();
  left_path_pub_->on_activate();
  obstacle_points_pub_->on_activate();

  static_costmap_pub_->on_activate();
  costmap_pub_->on_activate();
  target_path_pub_->on_activate();
  target_people_pub_->on_activate();
  target_obstacles_pub_->on_activate();
  target_goal_pose_pub_->on_activate();
  current_pose_pub_->on_activate();
}

void CaBotPlanner::deactivate() {
  iteration_path_pub_->on_deactivate();
  right_path_pub_->on_deactivate();
  left_path_pub_->on_deactivate();
  obstacle_points_pub_->on_deactivate();

  static_costmap_pub_->on_deactivate();
  costmap_pub_->on_deactivate();
  target_path_pub_->on_deactivate();
  target_people_pub_->on_deactivate();
  target_obstacles_pub_->on_deactivate();
  target_goal_pose_pub_->on_deactivate();
  current_pose_pub_->on_deactivate();
}

void CaBotPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                             std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto plugins = costmap_ros->getLayeredCostmap()->getPlugins();
  for (auto plugin = plugins->begin(); plugin != plugins->end(); plugin++) {
    RCLCPP_INFO(logger_, "Plugin: %s", plugin->get()->getName().c_str());
  }

  auto node = parent_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_DEBUG(logger_, "Configuring Cabot Planner: %s", name_.c_str());

  CaBotPlannerOptions defaultValue;
  PathEstimateOptions peDefaultValue;

  declare_parameter_if_not_declared(node, name + ".optimize_distance_from_start",
                                    rclcpp::ParameterValue(defaultValue.optimize_distance_from_start));
  node->get_parameter(name + ".optimize_distance_from_start", options_.optimize_distance_from_start);

  declare_parameter_if_not_declared(node, name + ".initial_node_interval",
                                    rclcpp::ParameterValue(defaultValue.initial_node_interval));
  node->get_parameter(name + ".initial_node_interval", options_.initial_node_interval);

  declare_parameter_if_not_declared(node, name + ".gravity_factor",
                                    rclcpp::ParameterValue(defaultValue.gravity_factor));
  node->get_parameter(name + ".gravity_factor", options_.gravity_factor);

  declare_parameter_if_not_declared(node, name + ".link_spring_factor",
                                    rclcpp::ParameterValue(defaultValue.link_spring_factor));
  node->get_parameter(name + ".link_spring_factor", options_.link_spring_factor);

  declare_parameter_if_not_declared(node, name + ".anchor_spring_factor",
                                    rclcpp::ParameterValue(defaultValue.anchor_spring_factor));
  node->get_parameter(name + ".anchor_spring_factor", options_.anchor_spring_factor);

  declare_parameter_if_not_declared(node, name + ".obstacle_margin",
                                    rclcpp::ParameterValue(defaultValue.obstacle_margin));
  node->get_parameter(name + ".obstacle_margin", options_.obstacle_margin);

  declare_parameter_if_not_declared(node, name + ".fix_node", rclcpp::ParameterValue(defaultValue.fix_node));
  node->get_parameter(name + ".fix_node", options_.fix_node);

  declare_parameter_if_not_declared(node, name + ".adjust_start", rclcpp::ParameterValue(defaultValue.adjust_start));
  node->get_parameter(name + ".adjust_start", options_.adjust_start);

  declare_parameter_if_not_declared(node, name + ".use_navcog_path_on_failure", rclcpp::ParameterValue(defaultValue.use_navcog_path_on_failure));
  node->get_parameter(name + ".use_navcog_path_on_failure", options_.use_navcog_path_on_failure);

  declare_parameter_if_not_declared(node, name + ".interim_plan_publish_interval",
                                    rclcpp::ParameterValue(defaultValue.interim_plan_publish_interval));
  node->get_parameter(name + ".interim_plan_publish_interval", options_.interim_plan_publish_interval);

  declare_parameter_if_not_declared(node, name + ".max_obstacle_scan_distance",
                                    rclcpp::ParameterValue(defaultValue.max_obstacle_scan_distance));
  node->get_parameter(name + ".max_obstacle_scan_distance", options_.max_obstacle_scan_distance);

  declare_parameter_if_not_declared(node, name + ".kdtree_search_radius",
                                    rclcpp::ParameterValue(defaultValue.kdtree_search_radius));
  node->get_parameter(name + ".kdtree_search_radius", options_.kdtree_search_radius);

  declare_parameter_if_not_declared(node, name + ".kdtree_max_results",
                                    rclcpp::ParameterValue(defaultValue.kdtree_max_results));
  node->get_parameter(name + ".kdtree_max_results", options_.kdtree_max_results);

  declare_parameter_if_not_declared(node, name + ".min_iteration_count",
                                    rclcpp::ParameterValue(defaultValue.min_iteration_count));
  node->get_parameter(name + ".min_iteration_count", options_.min_iteration_count);

  declare_parameter_if_not_declared(node, name + ".max_iteration_count",
                                    rclcpp::ParameterValue(defaultValue.max_iteration_count));
  node->get_parameter(name + ".max_iteration_count", options_.max_iteration_count);

  declare_parameter_if_not_declared(node, name + ".static_layer_name", rclcpp::ParameterValue("static_layer"));
  node->get_parameter(name + ".static_layer_name", static_layer_name_);

  declare_parameter_if_not_declared(node, name + ".inflation_layer_name", rclcpp::ParameterValue("inflation_layer"));
  node->get_parameter(name + ".inflation_layer_name", inflation_layer_name_);

  // private params

  declare_parameter_if_not_declared(node, name + ".private.iteration_scale_min",
                                    rclcpp::ParameterValue(defaultValue.iteration_scale_min));
  node->get_parameter(name + ".private.iteration_scale_min", options_.iteration_scale_min);

  declare_parameter_if_not_declared(node, name + ".priavte.iteration_scale_interval",
                                    rclcpp::ParameterValue(defaultValue.iteration_scale_interval));
  node->get_parameter(name + ".private.iteration_scale_interval", options_.iteration_scale_interval);

  declare_parameter_if_not_declared(node, name + ".private.iteration_scale_max",
                                    rclcpp::ParameterValue(defaultValue.iteration_scale_max));
  node->get_parameter(name + ".private.iteration_scale_max", options_.iteration_scale_max);

  declare_parameter_if_not_declared(node, name + ".private.complete_threshold",
                                    rclcpp::ParameterValue(defaultValue.complete_threshold));
  node->get_parameter(name + ".private.complete_threshold", options_.complete_threshold);

  declare_parameter_if_not_declared(node, name + ".private.min_distance_to_obstacle_cell",
                                    rclcpp::ParameterValue(defaultValue.min_distance_to_obstacle_cell));
  node->get_parameter(name + ".private.min_distance_to_obstacle_cell", options_.min_distance_to_obstacle_cell);

  declare_parameter_if_not_declared(node, name + ".private.min_distance_to_obstacle_group_cell",
                                    rclcpp::ParameterValue(defaultValue.min_distance_to_obstacle_group_cell));
  node->get_parameter(name + ".private.min_distance_to_obstacle_group_cell", options_.min_distance_to_obstacle_group_cell);

  declare_parameter_if_not_declared(node, name + ".private.min_anchor_length",
                                    rclcpp::ParameterValue(defaultValue.min_anchor_length));
  node->get_parameter(name + ".private.min_anchor_length", options_.min_anchor_length);

  declare_parameter_if_not_declared(node, name + ".private.min_link_length",
                                    rclcpp::ParameterValue(defaultValue.min_link_length));
  node->get_parameter(name + ".private.min_link_length", options_.min_link_length);

  declare_parameter_if_not_declared(node, name + ".private.go_around_detect_threshold",
                                    rclcpp::ParameterValue(defaultValue.go_around_detect_threshold));
  node->get_parameter(name + ".private.go_around_detect_threshold", options_.go_around_detect_threshold);

  // params for path width estimation

  declare_parameter_if_not_declared(node, name + ".path_adjusted_center",
                                    rclcpp::ParameterValue(peDefaultValue.path_adjusted_center));
  node->get_parameter(name + ".path_adjusted_center", pe_options_.path_adjusted_center);

  declare_parameter_if_not_declared(node, name + ".path_adjusted_minimum_path_width",
                                    rclcpp::ParameterValue(peDefaultValue.path_adjusted_minimum_path_width));
  node->get_parameter(name + ".path_adjusted_minimum_path_width", pe_options_.path_adjusted_minimum_path_width);

  declare_parameter_if_not_declared(node, name + ".path_width",
                                    rclcpp::ParameterValue(peDefaultValue.path_width));
  node->get_parameter(name + ".path_width", pe_options_.path_width);

  declare_parameter_if_not_declared(node, name + ".robot_radius",
                                    rclcpp::ParameterValue(peDefaultValue.robot_radius));
  node->get_parameter(name + ".robot_radius", pe_options_.robot_radius);

  declare_parameter_if_not_declared(node, name + ".safe_margin",
                                    rclcpp::ParameterValue(peDefaultValue.safe_margin));
  node->get_parameter(name + ".safe_margin", pe_options_.safe_margin);

  declare_parameter_if_not_declared(node, name + ".path_length_to_width_factor",
                                    rclcpp::ParameterValue(peDefaultValue.path_length_to_width_factor));
  node->get_parameter(name + ".path_length_to_width_factor", pe_options_.path_length_to_width_factor);


  if (costmap_capture_ != nullptr) {
    delete costmap_capture_;
  }
  costmap_capture_ = new CostmapLayerCapture(costmap_ros->getLayeredCostmap(), {});
  if (static_costmap_capture_ != nullptr) {
    delete static_costmap_capture_;
  }
  static_costmap_capture_ =
      new CostmapLayerCapture(costmap_ros->getLayeredCostmap(), {static_layer_name_, inflation_layer_name_});

  declare_parameter_if_not_declared(node, name + ".path_topic", rclcpp::ParameterValue("/path"));
  node->get_parameter(name + ".path_topic", path_topic_);

  rclcpp::QoS path_qos(10);
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      path_topic_, path_qos, std::bind(&CaBotPlanner::pathCallback, this, std::placeholders::_1));

  declare_parameter_if_not_declared(node, name + ".path_debug", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".path_debug", path_debug_);
  if (path_debug_) {
    declare_parameter_if_not_declared(node, name + ".iteration_path_topic", rclcpp::ParameterValue("/iteration_path"));
    node->get_parameter(name + ".iteration_path_topic", iteration_path_topic_);
    declare_parameter_if_not_declared(node, name + ".right_path_topic", rclcpp::ParameterValue("/right_path"));
    node->get_parameter(name + ".right_path_topic", right_path_topic_);
    declare_parameter_if_not_declared(node, name + ".left_path_topic", rclcpp::ParameterValue("/left_path"));
    node->get_parameter(name + ".left_path_topic", left_path_topic_);
    declare_parameter_if_not_declared(node, name + ".obstacles_points_topic",
                                      rclcpp::ParameterValue("/obstacle_points"));
    node->get_parameter(name + ".obstacles_points_topic", obstacle_points_topic_);
    iteration_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(iteration_path_topic_, path_qos);
    right_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(right_path_topic_, path_qos);
    left_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(left_path_topic_, path_qos);
    obstacle_points_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud>(obstacle_points_topic_, path_qos);

    declare_parameter_if_not_declared(node, name + ".static_costmap_topic",
                                      rclcpp::ParameterValue("/debug/static_costmap"));
    node->get_parameter(name + ".static_costmap_topic", static_costmap_topic_);
    declare_parameter_if_not_declared(node, name + ".costmap_topic", rclcpp::ParameterValue("/debug/costmap"));
    node->get_parameter(name + ".costmap_topic", costmap_topic_);
    declare_parameter_if_not_declared(node, name + ".target_path_topic", rclcpp::ParameterValue("/debug/target_path"));
    node->get_parameter(name + ".target_path_topic", target_path_topic_);
    declare_parameter_if_not_declared(node, name + ".target_people_topic",
                                      rclcpp::ParameterValue("/debug/target_people"));
    node->get_parameter(name + ".target_people_topic", target_people_topic_);
    declare_parameter_if_not_declared(node, name + ".target_obstacles_topic",
                                      rclcpp::ParameterValue("/debug/target_obstacles"));
    node->get_parameter(name + ".target_obstacles_topic", target_obstacles_topic_);
    declare_parameter_if_not_declared(node, name + ".target_goal_pose_topic",
                                      rclcpp::ParameterValue("/debug/target_goal"));
    node->get_parameter(name + ".target_goal_pose_topic", target_goal_pose_topic_);
    declare_parameter_if_not_declared(node, name + ".current_pose_pub_topic",
                                      rclcpp::ParameterValue("/debug/current_pose"));
    node->get_parameter(name + ".current_pose_pub_topic", current_pose_pub_topic_);
    static_costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(static_costmap_topic_, path_qos);
    costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, path_qos);
    target_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(target_path_topic_, path_qos);
    target_people_pub_ = node->create_publisher<people_msgs::msg::People>(target_people_topic_, path_qos);
    target_obstacles_pub_ = node->create_publisher<people_msgs::msg::People>(target_obstacles_topic_, path_qos);
    target_goal_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(target_goal_pose_topic_, path_qos);
    current_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_pub_topic_, path_qos);
  }

  callback_handler_ =
      node->add_on_set_parameters_callback(std::bind(&CaBotPlanner::param_set_callback, this, std::placeholders::_1));

  declare_parameter_if_not_declared(node, name + ".odom_topic", rclcpp::ParameterValue("/odom"));
  node->get_parameter(name + ".odom_topic", odom_topic_);
  rclcpp::QoS odom_qos(10);
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos, std::bind(&CaBotPlanner::odomCallback, this, std::placeholders::_1));

  declare_parameter_if_not_declared(node, name + ".people_topic", rclcpp::ParameterValue("/people"));
  node->get_parameter(name + ".people_topic", people_topic_);
  declare_parameter_if_not_declared(node, name + ".obstacles_topic", rclcpp::ParameterValue("/obstacles"));
  node->get_parameter(name + ".obstacles_topic", obstacles_topic_);
  declare_parameter_if_not_declared(node, name + ".queue_topic", rclcpp::ParameterValue("/queue_people_py/queue"));
  node->get_parameter(name + ".queue_topic", queue_topic_);
  rclcpp::QoS people_qos(10);
  people_sub_ = node->create_subscription<people_msgs::msg::People>(
      people_topic_, people_qos, std::bind(&CaBotPlanner::peopleCallback, this, std::placeholders::_1));
  obstacles_sub_ = node->create_subscription<people_msgs::msg::People>(
      obstacles_topic_, people_qos, std::bind(&CaBotPlanner::obstaclesCallback, this, std::placeholders::_1));
  queue_sub_ = node->create_subscription<queue_msgs::msg::Queue>(
      queue_topic_, people_qos, std::bind(&CaBotPlanner::queueCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CaBotPlanner::param_set_callback(const std::vector<rclcpp::Parameter> params) {
  auto node = parent_.lock();

  auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  for (auto &&param : params) {
    if (!node->has_parameter(param.get_name())) {
      continue;
    }
    RCLCPP_DEBUG(logger_, "change param %s", param.get_name().c_str());

    if (param.get_name() == name_ + ".optimize_distance_from_start") {
      options_.optimize_distance_from_start = param.as_double();
    }
    if (param.get_name() == name_ + ".initial_node_interval") {
      options_.initial_node_interval = param.as_double();
    }
    if (param.get_name() == name_ + ".gravity_factor") {
      options_.gravity_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".link_spring_factor") {
      options_.link_spring_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".anchor_spring_factor") {
      options_.anchor_spring_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".obstacle_margin") {
      options_.obstacle_margin = param.as_double();
    }
    if (param.get_name() == name_ + ".fix_node") {
      options_.fix_node = param.as_bool();
    }
    if (param.get_name() == name_ + ".adjust_start") {
      options_.adjust_start = param.as_bool();
    }
    if (param.get_name() == name_ + ".use_navcog_path_on_failure") {
      options_.use_navcog_path_on_failure = param.as_bool();
    }
    if (param.get_name() == name_ + ".interim_plan_publish_interval") {
      options_.interim_plan_publish_interval = param.as_int();
    }

    if (param.get_name() == name_ + ".max_obstacle_scan_distance") {
      options_.max_obstacle_scan_distance = param.as_double();
    }
    if (param.get_name() == name_ + ".kdtree_search_radius") {
      options_.kdtree_search_radius = param.as_double();
    }
    if (param.get_name() == name_ + ".kdtree_max_results") {
      options_.kdtree_max_results = param.as_int();
    }
    if (param.get_name() == name_ + ".min_iteration_count") {
      options_.min_iteration_count = param.as_int();
    }
    if (param.get_name() == name_ + ".max_iteration_count") {
      options_.max_iteration_count = param.as_int();
    }

    // private
    if (param.get_name() == name_ + ".private.iteration_scale_min") {
      options_.iteration_scale_min = param.as_double();
    }
    if (param.get_name() == name_ + ".private.iteration_scale_interval") {
      options_.iteration_scale_interval = param.as_double();
    }
    if (param.get_name() == name_ + ".private.iteration_scale_max") {
      options_.iteration_scale_max = param.as_double();
    }
    if (param.get_name() == name_ + ".private.complete_threshold") {
      options_.complete_threshold = param.as_double();
    }
    if (param.get_name() == name_ + ".private.min_distance_to_obstacle_cell") {
      options_.min_distance_to_obstacle_cell = param.as_double();
    }
    if (param.get_name() == name_ + ".private.min_distance_to_obstacle_group_cell") {
      options_.min_distance_to_obstacle_group_cell = param.as_double();
    }
    if (param.get_name() == name_ + ".private.min_anchor_length") {
      options_.min_anchor_length = param.as_double();
    }
    if (param.get_name() == name_ + ".private.min_link_length") {
      options_.min_link_length = param.as_double();
    }
    if (param.get_name() == name_ + ".private.go_around_detect_threshold") {
     options_.go_around_detect_threshold = param.as_double();
    }

    // path width estimation
    if (param.get_name() == name_ + ".path_adjusted_center") {
      pe_options_.path_adjusted_center = param.as_double();
    }
    if (param.get_name() == name_ + ".path_adjusted_minimum_path_width") {
      pe_options_.path_adjusted_minimum_path_width = param.as_double();
    }
    if (param.get_name() == name_ + ".path_width") {
      pe_options_.path_width = param.as_double();
    }
    if (param.get_name() == name_ + ".robot_radius") {
      pe_options_.robot_radius = param.as_double();
    }
    if (param.get_name() == name_ + ".safe_margin") {
      pe_options_.safe_margin = param.as_double();
    }
    if (param.get_name() == name_ + ".path_length_to_width_factor") {
      pe_options_.path_length_to_width_factor = param.as_double();
    }
  }
  results->successful = true;
  return *results;
}

// public functions

nav_msgs::msg::Path CaBotPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                             const geometry_msgs::msg::PoseStamped &goal) {
  if (path_debug_) {
    auto path = nav_msgs::msg::Path();
    path.header.frame_id = "map";
    right_path_pub_->publish(path);
    left_path_pub_->publish(path);
    iteration_path_pub_->publish(path);
    auto pc = sensor_msgs::msg::PointCloud();
    pc.header.frame_id = "map";
    obstacle_points_pub_->publish(pc);
  }
  if (navcog_path_ == nullptr) {
    RCLCPP_DEBUG(logger_, "navcog path is null");
    return nav_msgs::msg::Path();
  }
  if (static_costmap_capture_ == nullptr) {
    RCLCPP_DEBUG(logger_, "static layer capture is null");
    return nav_msgs::msg::Path();
  }

  // lock while preparing data
  std::unique_lock<std::recursive_mutex> planner_lock(mutex_);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap_ros_->getCostmap()->getMutex()));
  costmap_capture_->capture();
  static_costmap_capture_->capture();
  costmap_lock.unlock();

  CaBotPlannerParam param(options_, pe_options_, start, goal, *navcog_path_, last_people_, last_obstacles_, last_queue_,
                          costmap_capture_->getCostmap(), static_costmap_capture_->getCostmap());
  planner_lock.unlock();

  // this should be okay for multithreading
  return createPlan(param);
}

nav_msgs::msg::Path CaBotPlanner::createPlan(CaBotPlannerParam &param) {
  auto t0 = std::chrono::system_clock::now();

  if (param.adjustPath() == false) return nav_msgs::msg::Path();

  CaBotPlan plans[] = {CaBotPlan(param), CaBotPlan(param), CaBotPlan(param)};
  CaBotPlan *plan = &plans[0];

  // find obstacles near the path
  param.findObstacles(plan->getTargetNodes());

  auto t1 = std::chrono::system_clock::now();
  auto ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  RCLCPP_INFO(logger_, "pre process duration: %ldms", ms1.count());

  if (path_debug_) {
    debug_output(param, *plan);
  }

  float complete_threshold = param.options.complete_threshold;
  int min_iteration_count = param.options.min_iteration_count;
  int max_iteration_count = param.options.max_iteration_count;
  std::chrono::duration<long int, std::ratio<1, 1000>> interim_plan_publish_interval(
      param.options.interim_plan_publish_interval);

  int total_count = 0;
  DetourMode modes[3] = {DetourMode::RIGHT, DetourMode::LEFT, DetourMode::IGNORE};
  rclcpp::Rate r(1);
  for (int i = 0; i < 3; i++) {
    CaBotPlan &plan = plans[i];
    plan.detour_mode = modes[i];

    int count = 0;
    int reduce = 0;
    while (rclcpp::ok()) {
      float total_diff = iterate(param, plan, count-reduce);
      //RCLCPP_INFO(logger_, "total_diff=%.3f, %ld, %.4f <> %.4f count=%d", total_diff, plan.nodes.size(),
      //             total_diff / plan.nodes.size(), complete_threshold, count-reduce);
      if (total_diff / plan.nodes.size() < complete_threshold && count > min_iteration_count) {
        RCLCPP_INFO(logger_, "total_diff=%.3f, %ld, %.4f <> %.4f count=%d", total_diff, plan.nodes.size(),
                     total_diff / plan.nodes.size(), complete_threshold, count);
        break;
      }
      if (total_diff / plan.nodes.size() > complete_threshold * 2) {
        //reduce += (count-reduce)/2;
      }
      count++;
      total_count++;
      if (count >= max_iteration_count) {
        break;
      }
      plan.adjustNodeInterval();
      if (checkGoAround(param, plan)) {
        break;
      }

      if (path_debug_) {
        auto now = std::chrono::system_clock::now();
        if (now - last_iteration_path_published_ > interim_plan_publish_interval) {
          RCLCPP_DEBUG(logger_, "publish interim plan %ld", now.time_since_epoch().count());
          iteration_path_pub_->publish(plan.getPlan(false));
          last_iteration_path_published_ = now;
        }
      }
      if (rcutils_logging_logger_is_enabled_for(logger_.get_name(), RCUTILS_LOG_SEVERITY_DEBUG)) {
        r.sleep();
      }
    }

    RCLCPP_DEBUG(logger_, "less than the threshold and completed");
    // if converged path collides with obstacle, change detoure mode
    bool okay = checkPath(param, plan);
    plan.okay = okay;
    if (i == 1) {
      if (plans[0].okay || plans[1].okay) {
        break;
      }
    }
  }
  if (path_debug_) {
    right_path_pub_->publish(plans[0].getPlan(false));
    left_path_pub_->publish(plans[1].getPlan(false));
  }

  if (plans[0].okay && plans[1].okay) {
    if (plans[0].length() <= plans[1].length()) {
      plan = &plans[0];
    } else {
      plan = &plans[1];
    }
  } else if (plans[0].okay) {
    plan = &plans[0];
  } else if (plans[1].okay) {
    plan = &plans[1];
  } else {
    plan = &plans[2];
  }

  auto t2 = std::chrono::system_clock::now();
  auto ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  RCLCPP_INFO(logger_, "Total iteration count = %d: duration: %ldms", total_count, ms2.count());
  auto ms3 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0);
  RCLCPP_INFO(logger_, "Total duration: %ldms", ms3.count());

  plan->okay = checkPath(param, *plan);
  if (plan->okay == false && options_.use_navcog_path_on_failure == false) {
    return nav_msgs::msg::Path();
  }
  auto planned_path = plan->getPlan(true);
  RCLCPP_INFO(logger_, "plan size = %ld, nodes size = %ld", planned_path.poses.size(), plan->nodes.size());
  return planned_path;
}

// prepare navcog path by topic
void CaBotPlanner::pathCallback(nav_msgs::msg::Path::SharedPtr path) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  RCLCPP_DEBUG(logger_, "received navcog path");
  navcog_path_ = path;
}

void CaBotPlanner::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received odom");
  last_odom_ = odom;
}

void CaBotPlanner::peopleCallback(people_msgs::msg::People::SharedPtr people) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received people %ld", people->people.size());
  last_people_ = people;
}

void CaBotPlanner::obstaclesCallback(people_msgs::msg::People::SharedPtr obstacles) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received obstacles %ld", obstacles->people.size());
  last_obstacles_ = obstacles;
}

void CaBotPlanner::queueCallback(queue_msgs::msg::Queue::SharedPtr queue) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received queue %ld", queue->people.size());
  last_queue_ = queue;
}

float CaBotPlanner::iterate(const CaBotPlannerParam &param, CaBotPlan &plan, int count) {
  float scale = std::min(param.options.iteration_scale_max,
                         param.options.iteration_scale_min + param.options.iteration_scale_interval * count);

  float gravity_factor = std::pow(param.options.gravity_factor / param.resolution, 2);
  float link_spring_factor = param.options.link_spring_factor / param.resolution;
  float anchor_spring_factor = param.options.anchor_spring_factor;

  float obstacle_margin_cell = param.options.obstacle_margin / param.resolution; // convert to cell
  float min_distance_to_obstacle_cell = param.options.min_distance_to_obstacle_cell;
  float min_distance_to_obstacle_group_cell = param.options.min_distance_to_obstacle_group_cell;
  float min_anchor_length = param.options.min_anchor_length;
  float min_link_length = param.options.min_link_length;

  RCLCPP_DEBUG(logger_, "iteration scale=%.4f", scale);

  std::vector<Node> newNodes;
  for (unsigned long i = 1; i < plan.nodes.size(); i++) {
    Node *n0 = &plan.nodes[i - 1];
    Node *n1 = &plan.nodes[i];
    Node newNode;
    newNode.x = n0->x;
    newNode.y = n0->y;
    if (options_.fix_node) {
      newNode.fixed = n0->fixed;
    }
    if (i < plan.nodes.size() - 1) {
      Node *n2 = &plan.nodes[i + 1];
      n1->angle = normalized_diff((*n2 - *n1).yaw(), (*n0 - *n1).yaw());
    }
    newNodes.push_back(newNode);
  }
  RCLCPP_DEBUG(logger_, "nodes %ld/%ld", newNodes.size(), plan.nodes.size());

  bool collision = false;
  for (unsigned long i = plan.start_index + 1; i < plan.nodes.size() && i < plan.end_index; i++) {
    Node *n1 = &plan.nodes[i];
    // gravity term with obstacles that the path collide
    std::vector<ObstacleGroup>::const_iterator ogit;
    for (ogit = param.groups.begin(); ogit != param.groups.end(); ++ogit) {
      float d = ogit->distance(*n1);
      d = d - ogit->getSize(*n1);
      if (d < 0.0f) {
        collision = true;
      }
    }
  }
  // Compute forces for each node.
  // This section only depends on nodes_ and i, so it can be processed parallely
  // OMP_NUM_THREADS environment variable can change the number of threads
  // ~4 could be useful, too many threads can cause too much overhead
  // usually, this section takes a few milli seconds
  //#pragma omp parallel for schedule(dynamic, 1)
  for (unsigned long i = plan.start_index + 1; i < newNodes.size() && i < plan.end_index; i++) {
    Node *n0 = &plan.nodes[i - 1];
    Node *n1 = &plan.nodes[i];
    Node *newNode = &newNodes[i];

    if (newNode->fixed) {
      continue;
    }

    auto direction = (*n1 - *n0).yaw();
    auto right_yaw = (*n1 - *n0).yaw(-M_PI_2);
    auto left_yaw = (*n1 - *n0).yaw(+M_PI_2);

    // gravity term with obstacles that the path collide
    std::vector<ObstacleGroup>::const_iterator ogit;
    for (ogit = param.groups.begin(); ogit != param.groups.end(); ++ogit) {
      if (plan.detour_mode == DetourMode::IGNORE) {
        continue;
      }
      float d = ogit->distance(*n1);
      float size = ogit->getSize(*n1);
      float d2 = std::max(0.0f, d - size - obstacle_margin_cell);
      float yaw = 0;
      if (d2 < min_distance_to_obstacle_group_cell) {
        d2 = min_distance_to_obstacle_group_cell;
        if (plan.detour_mode == DetourMode::RIGHT) {
          yaw = right_yaw;
        } else if (plan.detour_mode == DetourMode::LEFT) {
          yaw = left_yaw;
        }
      } else {
        yaw = (*n1 - *ogit).yaw();
      }
      float m = gravity_factor / d2 / d2 * scale;
      newNode->move(yaw, m);
      // RCLCPP_INFO(logger_, "d = %.2f, size = %.2f, d2 = %.2f, gravity = %.2f, magnitude = %.2f, yaw = %.2f", d, size,
      // d2, gravity_factor, m, yaw);
    }

    // gravity term with other obstacles
    std::vector<Obstacle> list = param.getObstaclesNearPoint(*n1, collision);
    std::vector<Obstacle>::iterator it;
    for (it = list.begin(); it != list.end(); ++it) {
      if (it->lethal == nullptr) continue;
      float d = it->distance(*n1);

      //if (d < 0) continue;
      // d = std::max(min_distance_to_obstacle_cell, d - it->getSize());
      // float yaw =(*newNode - *it->lethal).yaw();

      float yaw = 0;
      if (d < 0) {
        d = std::max(min_distance_to_obstacle_cell, -d);
        yaw = (*it->lethal - *newNode).yaw();

        // if (plan.detour_mode == DetourMode::RIGHT) {
        //   yaw = right_yaw;
        // } else if (plan.detour_mode == DetourMode::LEFT) {
        //   yaw = left_yaw;
        // }
      } else {
        d = std::max(min_distance_to_obstacle_cell, d - it->getSize(*newNode));
        yaw = (*newNode - *it->lethal).yaw();
        auto yaw2 = (*it - *it->lethal).yaw();

        // need to separate collision
        //normalized_diff(yaw, yaw2);
        
        // if (std::abs(normalized_diff(yaw, yaw2)) < M_PI * 0.1) {
        //   if (plan.detour_mode == DetourMode::RIGHT) {
        //     yaw = right_yaw;
        //   } else {
        //     yaw =  left_yaw;
        //   }
        // }
        

      }

      float m = gravity_factor  / d / d * scale;
      newNode->move(yaw, m);
    }

    // spring term for anchors to original position
    float d = n1->distance(n1->anchor);
    if (d > min_anchor_length) {
      float yaw = (n1->anchor - *n1).yaw();
      newNode->move(yaw, d * anchor_spring_factor * scale);
    }

    // spring term with adjusent nodes
    if (i < plan.nodes.size() - 1) {
      Node *n2 = &plan.nodes[i + 1];

      d = n0->distance(*n1);
      if (d > min_link_length) {
        float yaw = (*n0 - *n1).yaw();
        newNode->move(yaw, d * link_spring_factor * scale);
        if (i > 1) {
          // make two adjucent link straight
          newNode->move(yaw - M_PI_2, tf2NormalizeAngle(M_PI - n0->angle) * link_spring_factor * scale);
          if (M_PI - abs(n0->angle) > M_PI_4) {
            RCLCPP_DEBUG(logger_, "n0->angle=%.2f, move(%.2f, %.2f)", n0->angle, yaw - M_PI_2,
                         tf2NormalizeAngle(M_PI - n0->angle) * link_spring_factor * scale);
          }
        }
      }
      // RCLCPP_DEBUG(logger_, "%d: %.2f %.2f ", i, d, yaw);
      d = n2->distance(*n1);
      if (d > min_link_length) {
        float yaw = (*n2 - *n1).yaw();
        newNode->move(yaw, d * link_spring_factor * scale);
        if (i < plan.nodes.size() - 2) {
          // make two adjucent link straight
          newNode->move(yaw + M_PI_2, tf2NormalizeAngle(M_PI - n2->angle) * link_spring_factor * scale);
          if (M_PI - abs(n2->angle) > M_PI_4) {
            RCLCPP_DEBUG(logger_, "n2->angle=%.2f, move(%.2f, %.2f)", n2->angle, yaw + M_PI_2,
                         tf2NormalizeAngle(M_PI - n2->angle) * link_spring_factor * scale);
          }
        }
      }
      // RCLCPP_DEBUG(logger_, "%d: %.2f %.2f", i, d, yaw);
      // RCLCPP_DEBUG(logger_, "(%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)", n0->x,
      // n0->y, n1->x, n1->y, newNode.x, newNode.y, n2->x, n2->y);
    }
  }

  // check if it is converged
  bool complete_flag = true;
  float total_diff = 0;
  for (unsigned long i = 0; i < newNodes.size(); i++) {
    float dx = newNodes[i].x - plan.nodes[i].x;
    float dy = newNodes[i].y - plan.nodes[i].y;

    float dist = std::hypot(dx, dy);
    total_diff += dist;
    plan.nodes[i].x = newNodes[i].x;
    plan.nodes[i].y = newNodes[i].y;
  }
  return total_diff;
}

bool CaBotPlanner::checkPath(const CaBotPlannerParam &param, CaBotPlan &plan) {
  for (unsigned long i = plan.start_index; (i < plan.nodes.size() - 1) && (i < plan.end_index-1); i++) {
    auto n0 = plan.nodes[i];
    auto n1 = plan.nodes[i + 1];

    int N = ceil(n0.distance(n1) / param.options.initial_node_interval);
    for (int j = 0; j < N; j++) {
      Point temp((n0.x * j + n1.x * (N - j)) / N, (n0.y * j + n1.y * (N - j)) / N);
      int index = param.getIndexByPoint(temp);

      if (plan.detour_mode == DetourMode::IGNORE) {
        if (index >= 0 && param.static_cost[index] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          float mx, my;
          param.mapToWorld(temp.x, temp.y, mx, my);
          RCLCPP_WARN(logger_, "ignore mode: path above threshold at (%.2f, %.2f)[%ld,%d] nodes.size=%ld", 
                      mx, my, i, j, plan.nodes.size());
          return false;
        }
      } else {
        if (index >= 0 && param.cost[index] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          float mx, my;
          param.mapToWorld(temp.x, temp.y, mx, my);
          RCLCPP_WARN(logger_, "path above threshold at (%.2f, %.2f)[%ld,%d] nodes.size=%ld", 
                      mx, my, i, j, plan.nodes.size());
          RCLCPP_ERROR(logger_, "something wrong (%.2f, %.2f), (%.2f, %.2f), %d", n0.x, n0.y, n1.x, n1.y, N);
          return false;
        }
      }
    }
  }
  return true;
}

bool CaBotPlanner::checkGoAround(const CaBotPlannerParam &param, CaBotPlan &plan) {
  float go_around_detect_threshold = param.options.go_around_detect_threshold;

  // check if the path makes a round
  float total_yaw_diff = 0;
  Node *n0 = &plan.nodes[0];
  Node *n1 = &plan.nodes[1];
  float prev_yaw = (*n1 - *n0).yaw();
  n0 = n1;
  for (unsigned long i = 2; i < plan.nodes.size(); i++) {
    n1 = &plan.nodes[i];
    float current_yaw = (*n1 - *n0).yaw();
    auto diff = normalized_diff(current_yaw, prev_yaw);
    total_yaw_diff += diff;
    //RCLCPP_INFO(logger_, "total yaw %.2f, diff %.2f, (%.2f, %.2f)", 
    //            total_yaw_diff, diff, current_yaw, prev_yaw);
    n0 = n1;
    prev_yaw = current_yaw;
    if (std::abs(total_yaw_diff) > go_around_detect_threshold) {
       RCLCPP_ERROR(logger_, "the path has more than two loops");
      return true;
    }
  }
  return false;
}

void CaBotPlanner::debug_output(const CaBotPlannerParam &param, CaBotPlan &plan) {
  // publish data required for path calcuration
  nav_msgs::msg::OccupancyGrid static_map_grid;
  static_map_grid.header.frame_id = costmap_ros_->getGlobalFrameID();
  static_map_grid.header.stamp = parent_.lock()->get_clock()->now();
  static_map_grid.info.width = param.width;
  static_map_grid.info.height = param.height;
  static_map_grid.info.resolution = param.resolution;
  static_map_grid.info.origin.position.x = param.origin_x;
  static_map_grid.info.origin.position.y = param.origin_y;
  static_map_grid.data.clear();
  for (int i = 0; i < param.width * param.height; i++) {
    static_map_grid.data.push_back(param.static_cost[i]);
  }
  static_costmap_pub_->publish(static_map_grid);
  nav_msgs::msg::OccupancyGrid map_grid;
  map_grid.header.frame_id = costmap_ros_->getGlobalFrameID();
  map_grid.header.stamp = parent_.lock()->get_clock()->now();
  map_grid.info.width = param.width;
  map_grid.info.height = param.height;
  map_grid.info.resolution = param.resolution;
  map_grid.info.origin.position.x = param.origin_x;
  map_grid.info.origin.position.y = param.origin_y;
  map_grid.data.clear();
  for (int i = 0; i < param.width * param.height; i++) {
    map_grid.data.push_back(param.cost[i]);
  }
  costmap_pub_->publish(map_grid);
  target_path_pub_->publish(param.navcog_path);
  target_people_pub_->publish(param.people_msg);
  target_obstacles_pub_->publish(param.obstacles_msg);
  target_goal_pose_pub_->publish(param.goal);
  current_pose_pub_->publish(param.start);

  // for debug
  sensor_msgs::msg::PointCloud pc;
  pc.header.frame_id = "map";
  for (auto oit = param.obstacles.begin(); oit != param.obstacles.end(); ++oit) {
    float wx, wy;
    param.mapToWorld(oit->x + 0.5, oit->y + 0.5, wx, wy);
    geometry_msgs::msg::Point32 point32;
    point32.x = wx;
    point32.y = wy;
    point32.z = (oit->is_static ? 0.0 : 2.0) + ((oit->cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ? 0.0 : 1.0);
    pc.points.push_back(point32);
  }

  std::vector<ObstacleGroup>::const_iterator ogit;
  for (ogit = param.groups.begin(); ogit != param.groups.end(); ++ogit) {
    float wx, wy;
    param.mapToWorld(ogit->x + 0.5, ogit->y + 0.5, wx, wy);
    RCLCPP_INFO(logger_, "group %.2f %.2f", ogit->x, ogit->y);
    geometry_msgs::msg::Point32 point32;
    point32.x = wx;
    point32.y = wy;
    point32.z = 2.0;
    pc.points.push_back(point32);

    std::set<Obstacle>::const_iterator oit;
    for (oit = ogit->obstacles_.begin(); oit != ogit->obstacles_.end(); oit++) {
      param.mapToWorld(oit->x + 0.25, oit->y + 0.25, wx, wy);
      geometry_msgs::msg::Point32 point32;
      point32.x = wx;
      point32.y = wy;
      point32.z = 3.0;
      pc.points.push_back(point32);
    }

    for (int i = 0; i < 360; i++) {
      auto p = Point(ogit->x + cos(M_PI * i / 180) * 1, ogit->y + sin(M_PI * i / 180) * 1);
      float d = ogit->getSize(p);
      param.mapToWorld(ogit->x + std::cos(M_PI * i / 180) * d, ogit->y + std::sin(M_PI * i / 180) * d, wx, wy);
      geometry_msgs::msg::Point32 point32;
      point32.x = wx;
      point32.y = wy;
      point32.z = 4.0;
      pc.points.push_back(point32);
    }

    for (unsigned long i = 0; i < ogit->hull_.size(); i++) {
      auto p = ogit->hull_.at(i);

      param.mapToWorld(p.x, p.y, wx, wy);
      geometry_msgs::msg::Point32 point32;
      point32.x = wx;
      point32.y = wy;
      point32.z = 5.0;
      pc.points.push_back(point32);
    }
  }
  obstacle_points_pub_->publish(pc);
}
}  // namespace cabot_navigation2
