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

#include <mutex>
#include "cabot_navigation2/cabot_planner.hpp"

#include <tf2/utils.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotPlanner, nav2_core::GlobalPlanner)

namespace cabot_navigation2 {

CaBotPlanner::CaBotPlanner(): 
iterate_counter_(0),
costmap_(nullptr),
static_layer_capture_(nullptr),
last_iteration_path_published_(std::chrono::system_clock::now()),
cost_(nullptr), 
static_cost_(nullptr),
mark_(nullptr), 
data_(nullptr), 
data_non_collision_(nullptr),
idx_(nullptr),
idx_non_collision_(nullptr)
{

}

CaBotPlanner::~CaBotPlanner() {
}

void CaBotPlanner::cleanup() {}

void CaBotPlanner::activate() {
  iteration_path_pub_->on_activate();
  right_path_pub_->on_activate();
  left_path_pub_->on_activate();
  obstacle_points_pub_->on_activate();
}

void CaBotPlanner::deactivate() {
  iteration_path_pub_->on_deactivate();
  right_path_pub_->on_deactivate();
  left_path_pub_->on_deactivate();
  obstacle_points_pub_->on_deactivate();
}

void CaBotPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                             std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();

  auto plugins = costmap_ros->getLayeredCostmap()->getPlugins();
  for(auto plugin = plugins->begin(); plugin != plugins->end(); plugin++) {
    RCLCPP_INFO(logger_, "Plugin: %s", plugin->get()->getName().c_str());
  }

  auto node = parent_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_DEBUG(logger_, "Configuring Cabot Planner: %s", name_.c_str());

  CaBotPlannerOptions defaultValue;

  declare_parameter_if_not_declared(node, name + ".initial_node_interval_meter", rclcpp::ParameterValue(defaultValue.initial_node_interval_meter));
  node->get_parameter(name + ".initial_node_interval_meter", options_.initial_node_interval_meter);

  declare_parameter_if_not_declared(node, name + ".devide_link_cell_interval_threshold", rclcpp::ParameterValue(defaultValue.devide_link_cell_interval_threshold));
  node->get_parameter(name + ".devide_link_cell_interval_threshold", options_.devide_link_cell_interval_threshold);

  declare_parameter_if_not_declared(node, name + ".optimize_distance_from_start", rclcpp::ParameterValue(defaultValue.optimize_distance_from_start));
  node->get_parameter(name + ".optimize_distance_from_start", options_.optimize_distance_from_start);

  declare_parameter_if_not_declared(node, name + ".iteration_scale", rclcpp::ParameterValue(defaultValue.iteration_scale));
  node->get_parameter(name + ".iteration_scale", options_.iteration_scale);

  declare_parameter_if_not_declared(node, name + ".iteration_scale_interval", rclcpp::ParameterValue(defaultValue.iteration_scale_interval));
  node->get_parameter(name + ".iteration_scale_interval", options_.iteration_scale_interval);

  declare_parameter_if_not_declared(node, name + ".gravity_factor", rclcpp::ParameterValue(defaultValue.gravity_factor));
  node->get_parameter(name + ".gravity_factor", options_.gravity_factor);

  declare_parameter_if_not_declared(node, name + ".link_spring_factor", rclcpp::ParameterValue(defaultValue.link_spring_factor));
  node->get_parameter(name + ".link_spring_factor", options_.link_spring_factor);

  declare_parameter_if_not_declared(node, name + ".link_straighten_factor", rclcpp::ParameterValue(defaultValue.link_straighten_factor));
  node->get_parameter(name + ".link_straighten_factor", options_.link_straighten_factor);

  declare_parameter_if_not_declared(node, name + ".anchor_spring_factor", rclcpp::ParameterValue(defaultValue.anchor_spring_factor));
  node->get_parameter(name + ".anchor_spring_factor", options_.anchor_spring_factor);

  declare_parameter_if_not_declared(node, name + ".complete_threshold", rclcpp::ParameterValue(defaultValue.complete_threshold));
  node->get_parameter(name + ".complete_threshold", options_.complete_threshold);

  declare_parameter_if_not_declared(node, name + ".obstacle_margin", rclcpp::ParameterValue(defaultValue.obstacle_margin));
  node->get_parameter(name + ".obstacle_margin", options_.obstacle_margin);

  declare_parameter_if_not_declared(node, name + ".min_distance_to_obstacle", rclcpp::ParameterValue(defaultValue.min_distance_to_obstacle));
  node->get_parameter(name + ".min_distance_to_obstacle", options_.min_distance_to_obstacle);

  declare_parameter_if_not_declared(node, name + ".min_distance_to_obstacle_group", rclcpp::ParameterValue(defaultValue.min_distance_to_obstacle_group));
  node->get_parameter(name + ".min_distance_to_obstacle_group", options_.min_distance_to_obstacle_group);

  declare_parameter_if_not_declared(node, name + ".min_anchor_length", rclcpp::ParameterValue(defaultValue.min_anchor_length));
  node->get_parameter(name + ".min_anchor_length", options_.min_anchor_length);

  declare_parameter_if_not_declared(node, name + ".min_link_length", rclcpp::ParameterValue(defaultValue.min_link_length));
  node->get_parameter(name + ".min_link_length", options_.min_link_length);

  declare_parameter_if_not_declared(node, name + ".go_around_detect_threshold", rclcpp::ParameterValue(defaultValue.go_around_detect_threshold));
  node->get_parameter(name + ".go_around_detect_threshold", options_.go_around_detect_threshold);

  declare_parameter_if_not_declared(node, name + ".cost_lethal_threshold", rclcpp::ParameterValue(defaultValue.cost_lethal_threshold));
  node->get_parameter(name + ".cost_lethal_threshold", options_.cost_lethal_threshold);

  declare_parameter_if_not_declared(node, name + ".cost_pass_threshold", rclcpp::ParameterValue(defaultValue.cost_pass_threshold));
  node->get_parameter(name + ".cost_pass_threshold", options_.cost_pass_threshold);

  declare_parameter_if_not_declared(node, name + ".max_obstacle_scan_distance", rclcpp::ParameterValue(defaultValue.max_obstacle_scan_distance));
  node->get_parameter(name + ".max_obstacle_scan_distance", options_.max_obstacle_scan_distance);

  declare_parameter_if_not_declared(node, name + ".interim_plan_publish_interval", rclcpp::ParameterValue(defaultValue.interim_plan_publish_interval));
  node->get_parameter(name + ".interim_plan_publish_interval", options_.interim_plan_publish_interval);

  declare_parameter_if_not_declared(node, name + ".kdtree_search_radius_in_cells", rclcpp::ParameterValue(defaultValue.kdtree_search_radius_in_cells));
  node->get_parameter(name + ".kdtree_search_radius_in_cells", options_.kdtree_search_radius_in_cells);

  declare_parameter_if_not_declared(node, name + ".kdtree_max_results", rclcpp::ParameterValue(defaultValue.kdtree_max_results));
  node->get_parameter(name + ".kdtree_max_results", options_.kdtree_max_results);

  declare_parameter_if_not_declared(node, name + ".max_iteration_count", rclcpp::ParameterValue(defaultValue.max_iteration_count));
  node->get_parameter(name + ".max_iteration_count", options_.max_iteration_count);

  declare_parameter_if_not_declared(node, name + ".fix_node", rclcpp::ParameterValue(defaultValue.fix_node));
  node->get_parameter(name + ".fix_node", options_.fix_node);

  declare_parameter_if_not_declared(node, name + ".static_layer_name", rclcpp::ParameterValue("static_layer"));
  node->get_parameter(name + ".static_layer_name", static_layer_name_);

  if (static_layer_capture_ != nullptr) {
    delete static_layer_capture_;
  }
  static_layer_capture_ = new CostmapLayerCapture(costmap_ros->getLayeredCostmap(), static_layer_name_);

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
    declare_parameter_if_not_declared(node, name + ".obstacles_points_topic", rclcpp::ParameterValue("/obstacle_points"));
    node->get_parameter(name + ".obstacles_points_topic", obstacle_points_topic_);
    iteration_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(iteration_path_topic_, path_qos);
    right_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(right_path_topic_, path_qos);
    left_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(left_path_topic_, path_qos);
    obstacle_points_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud>(obstacle_points_topic_, path_qos);
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
  rclcpp::QoS people_qos(10);
  people_sub_ = node->create_subscription<people_msgs::msg::People>(
    people_topic_, people_qos, std::bind(&CaBotPlanner::peopleCallback, this, std::placeholders::_1));
  obstacles_sub_ = node->create_subscription<people_msgs::msg::People>(
    obstacles_topic_, people_qos, std::bind(&CaBotPlanner::obstaclesCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CaBotPlanner::param_set_callback(const std::vector<rclcpp::Parameter> params) {
  auto node = parent_.lock();

  auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  for (auto &&param : params) {
    if (!node->has_parameter(param.get_name())) {
      continue;
    }
    RCLCPP_DEBUG(logger_, "change param %s", param.get_name().c_str());

    if (param.get_name() == name_ + ".initial_node_interval_meter") {
      options_.initial_node_interval_meter = param.as_double();
    }
    if (param.get_name() == name_ + ".devide_link_cell_interval_threshold") {
      options_.devide_link_cell_interval_threshold = param.as_double();
    }
    if (param.get_name() == name_ + ".optimize_distance_from_start") {
      options_.optimize_distance_from_start = param.as_double();
    }
    if (param.get_name() == name_ + ".iteration_scale") {
      options_.iteration_scale = param.as_double();
    }
    if (param.get_name() == name_ + ".iteration_scale_interval") {
      options_.iteration_scale_interval = param.as_double();
    }
    if (param.get_name() == name_ + ".gravity_factor") {
      options_.gravity_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".link_spring_factor") {
      options_.link_spring_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".link_straighten_factor") {
      options_.link_straighten_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".anchor_spring_factor") {
      options_.anchor_spring_factor = param.as_double();
    }
    if (param.get_name() == name_ + ".complete_threshold") {
      options_.complete_threshold = param.as_double();
    }
    if (param.get_name() == name_ + ".obstacle_margin") {
      options_.obstacle_margin = param.as_double();
    }
    if (param.get_name() == name_ + ".min_distance_to_obstacle") {
      options_.min_distance_to_obstacle = param.as_double();
    }
    if (param.get_name() == name_ + ".min_distance_to_obstacle_group") {
      options_.min_distance_to_obstacle_group = param.as_double();
    }
    if (param.get_name() == name_ + ".min_anchor_length") {
      options_.min_anchor_length = param.as_double();
    }
    if (param.get_name() == name_ + ".min_link_length") {
      options_.min_link_length = param.as_double();
    }
    if (param.get_name() == name_ + ".go_around_detect_threshold") {
      options_.go_around_detect_threshold = param.as_double();
    }

    if (param.get_name() == name_ + ".cost_lethal_threshold") {
      options_.cost_lethal_threshold = param.as_int();
    }
    if (param.get_name() == name_ + ".cost_pass_threshold") {
      options_.cost_pass_threshold = param.as_int();
    }
    if (param.get_name() == name_ + ".max_obstacle_scan_distance") {
      options_.max_obstacle_scan_distance = param.as_int();
    }
    if (param.get_name() == name_ + ".interim_plan_publish_interval") {
      options_.interim_plan_publish_interval = param.as_int();
    }
    if (param.get_name() == name_ + ".kdtree_search_radius_in_cells") {
      options_.kdtree_search_radius_in_cells = param.as_int();
    }
    if (param.get_name() == name_ + ".kdtree_max_results") {
      options_.kdtree_max_results = param.as_int();
    }
    if (param.get_name() == name_ + ".max_iteration_count") {
      options_.max_iteration_count = param.as_int();
    }
    if (param.get_name() == name_ + ".fix_node") {
      options_.fix_node = param.as_bool();
    }

    if (param.get_name() == name_ + ".path_debug") {
      path_debug_ = param.as_bool();
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
  if (static_layer_capture_ == nullptr) {
    RCLCPP_DEBUG(logger_, "static layer capture is null");
    return nav_msgs::msg::Path();
  }
  auto t0 = std::chrono::system_clock::now();

  RCLCPP_DEBUG(logger_, "navcog_path pose size %ld", navcog_path_->poses.size());
  nav_msgs::msg::Path path = normalizedPath(*navcog_path_);
  RCLCPP_DEBUG(logger_, "normalized path pose size %ld", path.poses.size());
  if (path.poses.empty()) {
    return nav_msgs::msg::Path();
  }
  for(unsigned long i = 0; i < path.poses.size(); i++){
    RCLCPP_DEBUG(logger_, "path[%3ld]=(%.2f, %.2f)", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
  }


  PathEstimateOptions pe_options;  
  pe_options.path_adjusted_center = 0.5;
  estimatePathWidthAndAdjust(path, costmap_, pe_options);
  //path = adjustedPathByStart(path, start);

  //path = adjustedPathByStart(path, start);
  path.poses.push_back(goal);
  RCLCPP_DEBUG(logger_, "adjusted path by start size %ld", path.poses.size());
  RCLCPP_DEBUG(logger_, "start=(%.2f, %.2f)", start.pose.position.x, start.pose.position.y);
  for(unsigned long i = 0; i < path.poses.size(); i++){
    RCLCPP_DEBUG(logger_, "path[%3ld]=(%.2f, %.2f)", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
  }

  RCLCPP_DEBUG(logger_, "start iteration: path pose size = %ld", path.poses.size());
  setParam(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
        costmap_->getOriginX(), costmap_->getOriginY(), costmap_->getResolution(), DetourMode::RIGHT);
  if (static_layer_capture_->capture()) {
    RCLCPP_INFO(logger_, "static layer is found");
  }

  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock1(*(costmap_->getMutex()));
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock2(*(static_layer_capture_->getCostmap()->getMutex()));
    setCost(costmap_->getCharMap(), static_layer_capture_->getCostmap()->getCharMap());
  }
  int scount = 0;
  for(int i = 0; i < width_*height_; i++) {
    if (static_cost_[i] > 0) {
      scount++;
    }
  }
  RCLCPP_INFO(logger_, "static layer lethal count = %d", scount);
  setPath(path);
  
  // find start/end index around the start
  float mx, my;
  worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  Node start_node(mx, my);
  unsigned long start_index = 0;
  unsigned long end_index = ULLONG_MAX;
  float min_dist = 1000;
  float optimize_distance_from_start = options_.optimize_distance_from_start / resolution_;
  float distance_from_start = 0;
  for (unsigned long i = 0; i < nodes_.size()-1; i++) {
    Node *n0 = &nodes_[i];
    if (start_node.distance(*n0)< min_dist) {
      start_index = i;
      min_dist = start_node.distance(*n0);
    }
  }
  for (unsigned long i = start_index; i < nodes_.size()-1; i++) {
    Node *n0 = &nodes_[i];
    Node *n1 = &nodes_[i+1];
    distance_from_start += n0->distance(*n1);
    end_index = i+1;
    if (optimize_distance_from_start < distance_from_start) {
      break;
    }
  }  
  if (start_index < (int)(5.0/options_.initial_node_interval_meter)) {
    start_index = 0;
  } else {
    start_index = start_index - (int)(5.0/options_.initial_node_interval_meter);
  }

  RCLCPP_INFO(logger_, "start_index=%ld, end_index=%ld", start_index, end_index);
  findObstacles(start_index, end_index);
  int count = 0;
  int sm = 0;
  int em = 0;
  rclcpp::Rate r(1);
  while (rclcpp::ok()) {
    bool result = iterate(start_index + sm, end_index + em);
    RCLCPP_DEBUG(logger_, "iterate count = %d, result=%d", count, result);
    count++;
    if (rcutils_logging_logger_is_enabled_for(logger_.get_name(), RCUTILS_LOG_SEVERITY_DEBUG)) {
      r.sleep();
    }
    if (result) {
      break;
    }

    int devide_link_cell_interval_threshold = options_.devide_link_cell_interval_threshold;
    for(unsigned long i = 0; i < nodes_.size()-1; i++) {
      Node *n0 = &nodes_[i];
      Node *n1 = &nodes_[i+1];

      auto distance = n0->distance(*n1);
      if (distance > devide_link_cell_interval_threshold) {
        Node newNode;
        newNode.x = (n0->x + n1->x)/2.0;
        newNode.y = (n0->y + n1->y)/2.0;
        newNode.anchor.x = (n0->anchor.x + n1->anchor.y) / 2.0;
        newNode.anchor.y = (n0->anchor.y + n1->anchor.y) / 2.0;
        newNode.angle = n0->angle;
        RCLCPP_INFO(logger_, "newNode2 [%ld] x = %.2f, y = %.2f", i+1, newNode.x, newNode.y);
        if (i < start_index) {
          sm ++;
        }
        if (i < end_index) {
          em ++;
        }
        i++;
        nodes_.insert(nodes_.begin() + i, newNode);
      }
    }
  }
  auto t1 = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  RCLCPP_INFO(logger_, "iteration count = %d: duration: %ldms", count, ms.count());

  auto plan = getPlan(true);
  RCLCPP_INFO(logger_, "plan size = %ld, nodes size = %ld", plan.poses.size(), nodes_.size());
  return plan;
}

// prepare navcog path by topic
void CaBotPlanner::pathCallback(nav_msgs::msg::Path::SharedPtr path) {
  navcog_path_ = path;
  RCLCPP_DEBUG(logger_, "received navcog path");
}

void CaBotPlanner::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received odom");

  last_odom_ = odom;
}

void CaBotPlanner::peopleCallback(people_msgs::msg::People::SharedPtr people) {
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received people %ld", people->people.size());

  last_people_ = people;
}

void CaBotPlanner::obstaclesCallback(people_msgs::msg::People::SharedPtr obstacles) {
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "received obstacles %ld", obstacles->people.size());

  last_obstacles_ = obstacles;
}

void CaBotPlanner::setParam(int width, int height, float origin_x, float origin_y, float resolution,
                            DetourMode detour) {
  RCLCPP_DEBUG(logger_, "setParam w=%d h=%d ox=%.2f oy=%.2f res=%.2f", width, height, origin_x, origin_y, resolution);
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  detour_ = detour;
  if (cost_) {
    delete cost_;
  }
  cost_ = new unsigned char[width_ * height_];
  if (static_cost_) {
    delete static_cost_;
  }
  static_cost_ = new unsigned char[width_ * height_];
  if (mark_) {
    delete mark_;
  }
  mark_ = new unsigned short[width_ * height_];
}

bool CaBotPlanner::worldToMap(float wx, float wy, float &mx, float &my) {
  mx = (wx - origin_x_) / resolution_;
  my = (wy - origin_y_) / resolution_;

  if (0 <= mx && mx < width_ && 0 <= my && my < height_) {
    return true;
  }
  return false;
}

void CaBotPlanner::mapToWorld(float mx, float my, float &wx, float &wy) {
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
}

int CaBotPlanner::getIndex(float x, float y) {
  int ix = static_cast<int>(x);
  int iy = static_cast<int>(y);
  if (ix < 0 || iy < 0 || width_ <= ix || height_ <= iy) {
    return -(ix + iy * width_);
  }
  return ix + iy * width_;
}

int CaBotPlanner::getIndexByPoint(Point &p) { return getIndex(p.x, p.y); }

void CaBotPlanner::setCost(unsigned char *cost, unsigned char *static_cost) {
  unsigned char *p1 = cost_;
  unsigned char *p2 = static_cost_;
  unsigned short *p3 = mark_;
  for (int x = 0; x < width_; x++) {
    for (int y = 0; y < height_; y++) {
      *p1++ = *cost++;
      *p2++ = *static_cost++;
      *p3++ = 0;
    }
  }

  double robot_velocity = 0;
  if (last_odom_ != nullptr) {
    robot_velocity = std::hypot(last_odom_->twist.twist.linear.x, last_odom_->twist.twist.linear.y);
  }
  if (last_people_ != nullptr) {
    for(auto it = last_people_->people.begin(); it != last_people_->people.end(); it++) {
      bool stationary = (std::find(it->tags.begin(), it->tags.end(), "stationary") != it->tags.end());

      // if robot is not moving, ignore all people
      if (robot_velocity < 0.1 && false) {
        clearCostAround(*it);
      } else {
        // if not ignore only stationary people
        if (!stationary) {
          clearCostAround(*it);
        }
      }
    }
  }
  if (last_obstacles_ != nullptr) {
    for(auto it = last_obstacles_->people.begin(); it != last_obstacles_->people.end(); it++) {
      bool stationary = (std::find(it->tags.begin(), it->tags.end(), "stationary") != it->tags.end());
      // ignore only stationary obstacles
      if (!stationary) {
        clearCostAround(*it);
      }
    }
  }
}

void CaBotPlanner::clearCostAround(people_msgs::msg::Person &person) {
  int cost_lethal_threshold = options_.cost_lethal_threshold;
  int max_obstacle_scan_distance = options_.max_obstacle_scan_distance;

  float mx, my;
  if (!worldToMap(person.position.x, person.position.y, mx, my)) {
    return;
  }
  RCLCPP_INFO(logger_, "clearCostAround %.2f %.2f \n %s", mx, my, rosidl_generator_traits::to_yaml(person).c_str());

  ObstacleGroup group;
  scanObstacleAt(group, mx, my, cost_lethal_threshold, max_obstacle_scan_distance);

  for(auto obstacle = group.obstacles_.begin(); obstacle != group.obstacles_.end(); obstacle++) {
    cost_[obstacle->index] = 0;
  }
}

void CaBotPlanner::setPath(nav_msgs::msg::Path path) {
  path_ = path;
  nodes_backup_ = getNodesFromPath(path_);
  //for(unsigned long i = 0; i < nodes_backup_.size(); i++) {
  //  RCLCPP_INFO(logger_, "%ld (%.2f %.2f)", i, nodes_backup_[i].x, nodes_backup_[i].y);
  //}
  resetNodes();
}

nav_msgs::msg::Path CaBotPlanner::getPlan(bool normalized, float normalize_length) {
  nav_msgs::msg::Path ret;
  ret.header.frame_id = "map";

  if (nodes_.size() == 0) {
    return ret;
  }

  if (normalized == false) {
    auto mp0 = nodes_[0];
    for (long unsigned int i = 1; i < nodes_.size(); i++) {
      auto mp1 = nodes_[i];
      float dy = mp1.y - mp0.y;
      float dx = mp1.x - mp0.x;
      if (dx == 0 && dy == 0) { continue; }
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);

      float mx, my;
      mapToWorld(mp0.x, mp0.y, mx, my);

      geometry_msgs::msg::PoseStamped wp;
      wp.pose.position.x = mx;
      wp.pose.position.y = my;
      wp.pose.orientation.x = q.x();
      wp.pose.orientation.y = q.y();
      wp.pose.orientation.z = q.z();
      wp.pose.orientation.w = q.w();
      ret.poses.push_back(wp);

      mp0 = mp1;
    }
  }
  else {
    auto mp0 = nodes_[0];
    for (long unsigned int i = 1; i < nodes_.size(); i++) {
      auto mp1 = nodes_[i];
      float dy = mp1.y - mp0.y;
      float dx = mp1.x - mp0.x;
      if (dx == 0 && dy == 0) { continue; }
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      
      float mx, my;
      mapToWorld(mp0.x, mp0.y, mx, my);

      geometry_msgs::msg::PoseStamped wp;
      wp.pose.position.x = mx;
      wp.pose.position.y = my;
      wp.pose.orientation.x = q.x();
      wp.pose.orientation.y = q.y();
      wp.pose.orientation.z = q.z();
      wp.pose.orientation.w = q.w();
      ret.poses.push_back(wp);

      float dist = std::hypot(dx, dy);
      if (dist > normalize_length / resolution_) {
        mp0 = Node(mp0.x + std::cos(yaw)*normalize_length / resolution_, mp0.y + std::sin(yaw)*normalize_length / resolution_);
        i--;
        continue;
      }

      mp0 = mp1;
    }

  }
  return ret;
}

bool CaBotPlanner::iterate(unsigned long start_index, unsigned long end_index) {
  float scale = std::max(options_.iteration_scale_interval, options_.iteration_scale - options_.iteration_scale_interval * iterate_counter_);
  float gravity_factor = options_.gravity_factor;
  float link_spring_factor = options_.link_spring_factor;
  float link_straighten_factor = options_.link_straighten_factor;
  float anchor_spring_factor = options_.anchor_spring_factor;
  float complete_threshold = options_.complete_threshold;
  float obstacle_margin = options_.obstacle_margin;
  float min_distance_to_obstacle = options_.min_distance_to_obstacle;
  float min_distance_to_obstacle_group = options_.min_distance_to_obstacle_group;
  float min_anchor_length = options_.min_anchor_length;
  float min_link_length = options_.min_link_length;
  float go_around_detect_threshold = options_.go_around_detect_threshold;
  int cost_pass_threshold = options_.cost_pass_threshold;
  std::chrono::duration<long int, std::ratio<1, 1000>> interim_plan_publish_interval(options_.interim_plan_publish_interval);
  iterate_counter_++;

  RCLCPP_DEBUG(logger_, "iteration scale=%.4f", scale);

  std::vector<Node> newNodes;
  for (unsigned long i = 1; i < nodes_.size(); i++) {
    Node *n0 = &nodes_[i - 1];
    Node *n1 = &nodes_[i];
    Node newNode;
    newNode.x = n0->x;
    newNode.y = n0->y;
    if (options_.fix_node) {
      newNode.fixed = n0->fixed;
    }
    if (i < nodes_.size() - 1) {
      Node *n2 = &nodes_[i+1];
      n1->angle = normalized_diff((*n2 - *n1).yaw(), (*n0 - *n1).yaw());
    }
    newNodes.push_back(newNode);
  }
  RCLCPP_DEBUG(logger_, "nodes %ld/%ld", newNodes.size(), nodes_.size());

  bool collision = false;
  for (unsigned long i = start_index+1; i < nodes_.size() && i < end_index; i++) {
    Node *n1 = &nodes_[i];
    // gravity term with obstacles that the path collide
    std::vector<ObstacleGroup>::iterator ogit;
    for (ogit = groups_.begin(); ogit != groups_.end(); ++ogit) {
      float d = ogit->distance(*n1);
      d = d - ogit->getSize(*n1);
      if ( d < 0.0f ){
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
  for (unsigned long i = start_index+1; i < newNodes.size() && i < end_index; i++) {
    Node *n0 = &nodes_[i - 1];
    Node *n1 = &nodes_[i];
    Node *newNode = &newNodes[i];

    if (newNode->fixed) {
      continue;
    }

    // gravity term with obstacles that the path collide
    std::vector<ObstacleGroup>::iterator ogit;
    for (ogit = groups_.begin(); ogit != groups_.end(); ++ogit) {
      if (detour_ == DetourMode::IGNORE) {
        continue;
      }
      float d = ogit->distance(*n1);
      float size = ogit->getSize(*n1);
      float d2 = std::max(0.0f, d - size - obstacle_margin);
      float yaw = 0;
      if (d2 < min_distance_to_obstacle_group) {
        d2 = min_distance_to_obstacle_group;
        if (detour_ == DetourMode::RIGHT) {
          yaw = (*n1 - *n0).yaw(-M_PI_2);
        } else {
          yaw = (*n1 - *n0).yaw(+M_PI_2);
        }
      } else {
        yaw = (*n1 - *ogit).yaw();
      }
      float m = gravity_factor / d2 / d2 * scale;
      newNode->move(yaw, m);
      //RCLCPP_DEBUG(logger_, "d = %.2f, size = %.2f, d2 = %.2f, gravity = %.2f, magnitude = %.2f, yaw = %.2f", d, size, d2, gravity_factor, m, yaw);
    }

    // gravity term with other obstacles
    std::vector<Obstacle> list = getObstaclesNearNode(*n1, collision);
    std::vector<Obstacle>::iterator it;
    for (it = list.begin(); it != list.end(); ++it) {
      float d = it->distance(*n1);
      d = std::max(0.0f, d - it->size);
      float yaw = 0;
      if (d < min_distance_to_obstacle) {
        d = min_distance_to_obstacle;
        if (detour_ == DetourMode::RIGHT) {
          yaw = (*n1 - *n0).yaw(-M_PI_2);
        } else {
          yaw = (*n1 - *n0).yaw(+M_PI_2);
        }
      } else {
        yaw = (*n1 - *it).yaw();
      }
      float m = gravity_factor / d / d * scale;
      newNode->move(yaw, m);
    }

    // spring term for anchors to original position
    float d = n1->distance(n1->anchor);
    if (d > min_anchor_length) {
      float yaw = (n1->anchor - *n1).yaw();
      newNode->move(yaw, d * anchor_spring_factor * scale);
    }

    // spring term with adjusent nodes
    if (i < nodes_.size() - 1) {
      Node *n2 = &nodes_[i + 1];

      d = n0->distance(*n1);
      if (d > min_link_length) {
        float yaw = (*n0 - *n1).yaw();
        newNode->move(yaw, d * link_spring_factor * scale);
        if (i > 1) {
          // make two adjucent link straight
          newNode->move(yaw - M_PI_2, tf2NormalizeAngle(M_PI-n0->angle) * link_straighten_factor * scale);
          if (M_PI - abs(n0->angle) > M_PI_4) {
            RCLCPP_DEBUG(logger_, "n0->angle=%.2f, move(%.2f, %.2f)",
                        n0->angle, yaw - M_PI_2, tf2NormalizeAngle(M_PI-n0->angle) * link_straighten_factor * scale);
          }
        }
      }
      // RCLCPP_DEBUG(logger_, "%d: %.2f %.2f ", i, d, yaw);
      d = n2->distance(*n1);
      if (d > min_link_length) {
        float yaw = (*n2 - *n1).yaw();
        newNode->move(yaw, d * link_spring_factor * scale);
        if (i < nodes_.size() - 2) {
          // make two adjucent link straight
          newNode->move(yaw + M_PI_2, tf2NormalizeAngle(M_PI-n2->angle) * link_straighten_factor * scale);
          if (M_PI - abs(n2->angle) > M_PI_4) {
            RCLCPP_DEBUG(logger_, "n2->angle=%.2f, move(%.2f, %.2f)",
                        n2->angle, yaw + M_PI_2, tf2NormalizeAngle(M_PI-n2->angle) * link_straighten_factor * scale);
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
    float dx = newNodes[i].x - nodes_[i].x;
    float dy = newNodes[i].y - nodes_[i].y;

    float dist = std::hypot(dx, dy);
    total_diff += dist;
    nodes_[i].x = newNodes[i].x;
    nodes_[i].y = newNodes[i].y;
  }
  if (total_diff / nodes_.size() > complete_threshold) {
    complete_flag = false;
  }
  //RCLCPP_DEBUG(logger_, "complete_flag=%d, total_diff=%.3f, %ld, %.4f <> %.4f", 
  //            complete_flag, total_diff, nodes_.size(), total_diff / nodes_.size(), complete_threshold);


  if (iterate_counter_ >= options_.max_iteration_count) {
    if (detour_ == DetourMode::RIGHT) {
        RCLCPP_ERROR(logger_, "cannot converge in %d iteration: right", iterate_counter_);
      } else if (detour_ == DetourMode::LEFT) {
        RCLCPP_ERROR(logger_, "cannot converge in %d iteration: left", iterate_counter_);
      } else if (detour_ == DetourMode::IGNORE) {
        RCLCPP_ERROR(logger_, "cannot converge in %d iteration: ignore", iterate_counter_);
      }
      complete_flag = true;
  }

  if (complete_flag) {
    RCLCPP_DEBUG(logger_, "less than the threshold and completed");
    // if converged path collides with obstacle, change detoure mode
    bool okay = checkPath(cost_pass_threshold);
    if (detour_ == DetourMode::RIGHT) {
      if (!okay) {
        if (path_debug_) {
          right_path_pub_->publish(getPlan());
        }
        detour_ = DetourMode::LEFT;
        resetNodes();
        RCLCPP_WARN(logger_, "right side cannot be passsed (%d)", iterate_counter_);
        iterate_counter_ = 0;
        return false;
      }
    } else if (detour_ == DetourMode::LEFT) {
      if (!okay) {
        if (path_debug_) {
          left_path_pub_->publish(getPlan());
        }
        detour_ = DetourMode::IGNORE;
        RCLCPP_WARN(logger_, "left side also cannot be passsed (%d)", iterate_counter_);
        resetNodes();
        iterate_counter_ = 0;
        return false;
      }
    } else if (detour_ == DetourMode::IGNORE) {
      if (!okay) {
        RCLCPP_WARN(logger_, "ignore mode may collide with obstacles (%d)", iterate_counter_);
        iterate_counter_ = 0;
        return true;
      }
    } else {
      RCLCPP_WARN(logger_, "unkown detour mode");
      iterate_counter_ = 0;
      return true;
    }
  }

  // check if the path makes a round
  float total_yaw_diff = 0;
  Node *n0 = &nodes_[0];
  Node *n1 = &nodes_[1];
  float prev_yaw = (*n1 - *n0).yaw();
  n0 = n1;
  for (unsigned long i = 2; i < nodes_.size(); i++) {
    n1 = &nodes_[i];
    float current_yaw = (*n1 - *n0).yaw();
    total_yaw_diff += normalized_diff(current_yaw, prev_yaw);
    n0 = n1;
    prev_yaw = current_yaw;
    if (std::abs(total_yaw_diff) > go_around_detect_threshold) {
      if (detour_ == DetourMode::RIGHT) {
        if (path_debug_) {
          right_path_pub_->publish(getPlan());
        }
        detour_ = DetourMode::LEFT;
        resetNodes();
        iterate_counter_ = 0;
        RCLCPP_INFO(logger_, "the path made a round: right");
        return false;
      } else if (detour_ == DetourMode::LEFT) {
        if (path_debug_) {
          left_path_pub_->publish(getPlan());
        }
        RCLCPP_INFO(logger_, "the path made a round: left");
        detour_ = DetourMode::IGNORE;
        resetNodes();
        iterate_counter_ = 0;
        return false;
      } else if (detour_ == DetourMode::IGNORE) {
        RCLCPP_INFO(logger_, "the path made a round: ignore");
        iterate_counter_ = 0;
        return true;
      } else {
        RCLCPP_INFO(logger_, "the path made a round: unknown detour mode");
        iterate_counter_ = 0;
        return true;
      }
    }
  }

  if (path_debug_) {
    auto now = std::chrono::system_clock::now();
    if (now - last_iteration_path_published_ > interim_plan_publish_interval) {
      RCLCPP_DEBUG(logger_, "publish interim plan %ld", now.time_since_epoch().count());
      iteration_path_pub_->publish(getPlan(true));
      last_iteration_path_published_ = now;
    }
  }
  if (complete_flag) {
    iterate_counter_ = 0;
  }
  return complete_flag;
}

bool CaBotPlanner::checkPath(int cost_threshold) {
  for (unsigned long i = 0; i < nodes_.size()-1; i++) {
    auto n0 = nodes_[i];
    auto n1 = nodes_[i+1];

    int N = ceil(n0.distance(n1) / resolution_);
    for(int i = 0; i < N; i++) {
      Point temp((n0.x * i + n1.x * (N - i))/N, (n0.y * i + n1.y * (N - i))/N);
      int index = getIndexByPoint(temp);

      if (index >= 0 && cost_[index] >= cost_threshold) {
        RCLCPP_INFO(logger_, "path above threshold at (%.2f, %.2f)", temp.x, temp.y);
        return false;
      }
    }
  }
  return true;
}

// protected methods
void CaBotPlanner::resetNodes() {
  nodes_.clear();
  for(unsigned long i = 0; i < nodes_backup_.size(); i++) {
    nodes_backup_[i].reset();
    nodes_.push_back(nodes_backup_[i]);
  }
}

void CaBotPlanner::scanObstacleAt(ObstacleGroup &group, float mx, float my, unsigned int min_obstacle_cost, float max_dist) {
  std::queue<std::pair<Obstacle, float>> queue;
  int index = getIndex(mx, my);
  if (index < 0) return;
  queue.push(std::pair(Obstacle(mx, my, index, 1), 0));

  while(queue.size() > 0) {
    auto entry = queue.front();
    auto temp = entry.first;
    queue.pop();
    //RCLCPP_DEBUG(logger_, "queue.size=%ld, cost=%d, mark=%d, (%.2f %.2f) [%d], %d", 
    //queue.size(), cost_[temp.index], mark_[temp.index], temp.x, temp.y, temp.index, entry.second);
    if (entry.second > max_dist) {
      continue;
    }
    if (mark_[temp.index] == 65535) {
      continue;
    }
    mark_[temp.index] = 65535;
    if (cost_[temp.index] < min_obstacle_cost || static_cost_[temp.index] >= min_obstacle_cost) {
      continue;
    }
    auto obstacle = Obstacle(temp.x, temp.y, temp.index, temp.size, true);
    obstacles_.insert(obstacle);
    group.add(obstacle);

    index = getIndex(temp.x+1, temp.y);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x+1, temp.y, index, 1), entry.second+1));
    }
    index = getIndex(temp.x+1, temp.y+1);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x+1, temp.y+1, index, 1), entry.second+sqrt(2)));
    }
    index = getIndex(temp.x, temp.y+1);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x, temp.y+1, index, 1), entry.second+1));
    }
    index = getIndex(temp.x-1, temp.y+1);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x-1, temp.y+1, index, 1), entry.second+sqrt(2)));
    }
    index = getIndex(temp.x-1, temp.y);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x-1, temp.y, index, 1), entry.second+1));
    }
    index = getIndex(temp.x-1, temp.y-1);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x-1, temp.y-1, index, 1), entry.second+sqrt(2)));
    }
    index = getIndex(temp.x, temp.y-1);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x, temp.y-1, index, 1), entry.second+1));
    }
    index = getIndex(temp.x+1, temp.y-1);
    if (index >= 0) {
      queue.push(std::pair(Obstacle(temp.x+1, temp.y-1, index, 1), entry.second+sqrt(2)));
    }
  }
}

/*
findObstacles scans the costmap to determine obstacles
  Obstacle class: point where the cost is higher than threthold
  ObstacleGroup class: points that all in a single adjucented region

1. find obstacle group collides with the path
2. find obstacles and groups near the path
3. merging nearby obstacle groups
4. create flann index for obstacles (not group obstacles) and point cloud for debug
*/
void CaBotPlanner::findObstacles(unsigned long start_index, unsigned long end_index) {
  // check obstacles only from current position to N meters forward
  int cost_lethal_threshold = options_.cost_lethal_threshold;
  int max_obstacle_scan_distance = options_.max_obstacle_scan_distance;

  groups_.clear();
  obstacles_.clear();

  // 1. find obstacle group collides with the path
  std::vector<int> marks;
  for (unsigned long i = start_index; i < nodes_.size() && i < end_index; i++) {
    int index = getIndexByPoint(nodes_[i]);
    if (index < 0) {
      continue;
    }
    if (mark_[index] > max_obstacle_scan_distance) {
      continue;
    }
    auto cost = cost_[index];
    if (cost < cost_lethal_threshold) {
      mark_[index] = 1;
      marks.push_back(index);
    }
    if (cost >= cost_lethal_threshold) {
      ObstacleGroup group;
      scanObstacleAt(group, nodes_[i].x, nodes_[i].y, cost_lethal_threshold, max_obstacle_scan_distance);
      if (group.complete()) {
        group.index = getIndexByPoint(group);
        group.collision = true;
        RCLCPP_INFO(logger_, "Group Obstacle %.2f %.2f %.2f %ld", group.x, group.y, group.size, group.obstacles_.size());
        groups_.push_back(group);
      }
    }
  }

  // 2. find obstacles and groups near the path
  unsigned long i = 0;
  long max_size = width_ * height_;
  while (i < marks.size()) {
    long index = marks[i++];

    unsigned char cost = cost_[index];
    unsigned char static_cost = static_cost_[index];
    unsigned short current = mark_[index];

    if (current > max_obstacle_scan_distance) { continue; }
    if (cost >= cost_lethal_threshold) {
      if (static_cost < cost_lethal_threshold) {
        float x = index % width_;
        float y = index / width_;
        ObstacleGroup group;
        scanObstacleAt(group, x, y, cost_lethal_threshold, max_obstacle_scan_distance);
        if (group.complete()) {
          group.index = getIndexByPoint(group);
          group.collision = false;
          RCLCPP_INFO(logger_, "Group Obstacle %.2f %.2f %.2f %ld", group.x, group.y, group.size, group.obstacles_.size());
          groups_.push_back(group);
        }
      } else {
        float x = index % width_;
        float y = index / width_;
        obstacles_.insert(Obstacle(x, y, index, 1, false));
      }
    } else {
      mark_[index] = 0;
    }

    if (0 <= index - 1 && mark_[index - 1] == 0) {
      mark_[index - 1] = current + 1;
      marks.push_back(index - 1);
    }
    if (index + 1 < max_size && mark_[index + 1] == 0) {
      mark_[index + 1] = current + 1;
      marks.push_back(index + 1);
    }
    if (0 <= index - width_ && mark_[index - width_] == 0) {
      mark_[index - width_] = current + 1;
      marks.push_back(index - width_);
    }
    if (index + width_ < max_size && mark_[index + width_] == 0) {
      mark_[index + width_] = current + 1;
      marks.push_back(index + width_);
    }
  }

  // 3. merging nearby obstacle groups
  bool flag = false;
  do {
    flag = false;
    if (groups_.size() > 1) {
      for (unsigned long i = 0; i < groups_.size()-1; i++) {
        if (groups_.back().distance(groups_.at(i)) < 5) {
          groups_.at(i).combine(groups_.back());
          groups_.pop_back();
          flag = true;
        }
      }
    }
  } while(flag);

  // 4. create flann index for obstacles (not group obstacles) and point cloud for debug
  unsigned long n = obstacles_.size();
  unsigned long n_collision = 0;
  for(auto obstacle = obstacles_.begin(); obstacle != obstacles_.end(); obstacle++) {
    if (obstacle->in_group) {
      n_collision++;
    }
  }
  if (idx_) {
    delete idx_;
    idx_ = nullptr;
  }
  if (idx_non_collision_) {
    delete idx_non_collision_;
    idx_non_collision_ = nullptr;
  }
  if (data_) {
    delete data_;
    data_ = nullptr;
  }
  if (data_non_collision_) {
    delete data_non_collision_;
    data_non_collision_ = nullptr;
  }
  RCLCPP_INFO(logger_, "data length=%ld, data non collision length=%ld", n, n-n_collision);
  if (n == 0 || n-n_collision == 0) {
    return;
  }
  data_ = new cv::Mat(n, 2, CV_32FC1);
  data_non_collision_ = new cv::Mat(n-n_collision, 2, CV_32FC1);
  olist_.clear();
  olist_non_collision_.clear();
  i = 0;
  unsigned long j = 0;
  sensor_msgs::msg::PointCloud pc;
  pc.header.frame_id = "map";
  // add index
  std::set<Obstacle>::iterator oit;
  for (oit = obstacles_.begin(); oit != obstacles_.end(); ++oit) {
    olist_.push_back(*oit);
    data_->at<float>(i, 0) = oit->x;
    data_->at<float>(i, 1) = oit->y;
    i++;
    if (!oit->in_group) {
      olist_non_collision_.push_back(*oit);
      data_non_collision_->at<float>(j, 0) = oit->x;
      data_non_collision_->at<float>(j, 1) = oit->y;
      j++;
    }

    float wx, wy;
    mapToWorld(oit->x+0.5, oit->y+0.5, wx, wy);
    geometry_msgs::msg::Point32 point32;
    point32.x = wx;
    point32.y = wy;
    point32.z = oit->in_group?0.0:1.0;
    pc.points.push_back(point32);
  }
  RCLCPP_DEBUG(logger_, "making index %ld/%ld", i, n);
  idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  RCLCPP_DEBUG(logger_, "making index %ld/%ld", j, n-n_collision);
  idx_non_collision_ = new cv::flann::Index(*data_non_collision_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  RCLCPP_DEBUG(logger_, "obstacles = %ld", obstacles_.size());
  // for debug
  std::vector<ObstacleGroup>::iterator ogit;
  for (ogit = groups_.begin(); ogit != groups_.end(); ++ogit) {
    float wx, wy;
    mapToWorld(ogit->x+0.5, ogit->y+0.5, wx, wy);
    RCLCPP_INFO(logger_, "group %.2f %.2f", ogit->x, ogit->y);
    geometry_msgs::msg::Point32 point32;
    point32.x = wx;
    point32.y = wy;
    point32.z = 2.0;
    pc.points.push_back(point32);

    std::set<Obstacle>::iterator oit;
    for(oit = ogit->obstacles_.begin(); oit != ogit->obstacles_.end(); oit++) {
      mapToWorld(oit->x+0.25, oit->y+0.25, wx, wy);
      geometry_msgs::msg::Point32 point32;
      point32.x = wx;
      point32.y = wy;
      point32.z = 3.0;
      pc.points.push_back(point32);
    }

    for(int i = 0; i < 360; i++) {
      auto p = Point(ogit->x + cos(M_PI*i/180) * 1, ogit->y + sin(M_PI*i/180) * 1);
      float d = ogit->getSize(p);
      mapToWorld(ogit->x + std::cos(M_PI*i/180)*d, ogit->y+std::sin(M_PI*i/180)*d, wx, wy);
      geometry_msgs::msg::Point32 point32;
      point32.x = wx;
      point32.y = wy;
      point32.z = 4.0;
      pc.points.push_back(point32);
    }

    for(unsigned long i = 0; i < ogit->hull_.size(); i++) {
      auto p = ogit->hull_.at(i);

      mapToWorld(p.x, p.y, wx, wy);
      geometry_msgs::msg::Point32 point32;
      point32.x = wx;
      point32.y = wy;
      point32.z = 5.0;
      pc.points.push_back(point32);
    }
  }
  if (path_debug_) {
    obstacle_points_pub_->publish(pc);
  }
}

std::vector<Obstacle> CaBotPlanner::getObstaclesNearNode(Node &node, bool collision) {
  int kdtree_search_radius_in_cells = options_.kdtree_search_radius_in_cells;
  int kdtree_max_results = options_.kdtree_max_results;

  std::vector<Obstacle> list;
  cv::flann::Index * idx = collision ? idx_non_collision_: idx_;
  if (!idx || !data_) {
    return list;
  }

  cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
  query.at<float>(0) = node.x;
  query.at<float>(1) = node.y;
  std::vector<int> indices;
  std::vector<float> dists;

  int m = idx->radiusSearch(query, indices, dists, kdtree_search_radius_in_cells, kdtree_max_results);
  for (int i = 0; i < m; i++) {
    if (collision) {
      list.push_back(olist_non_collision_[indices[i]]);
    } else {
      list.push_back(olist_[indices[i]]);
    }
  }
  return list;
}

/*
 * generate map coordinate points
 */
std::vector<Node> CaBotPlanner::getNodesFromPath(nav_msgs::msg::Path path) {
  std::vector<Node> nodes;

  auto initial_node_interval_meter = options_.initial_node_interval_meter;

  // push initial pose
  auto p0 = path.poses[0].pose.position;
  for (long unsigned int i = 1; i < path.poses.size(); i++) {
    auto p1 = path.poses[i].pose.position;

    auto dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
    int N = std::round(dist / initial_node_interval_meter);
    if (N == 0) continue;

    for (int j = 0; j <= N; j++) {
      // deal with the last pose
      if (j == N && i < path.poses.size() - 1) {
        continue;
      }
      float mx = 0, my = 0;
      worldToMap((p0.x * (N - j) + p1.x * j) / N, (p0.y * (N - j) + p1.y * j) / N, mx, my);
      Node temp = Node(mx, my);
      temp.fixed = (j == 0) || (j == N);
      nodes.push_back(temp);
    }
    p0 = p1;
  }

  return nodes;
}

}  // namespace cabot_navigation2
