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
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CaBotPlanner, nav2_core::GlobalPlanner)

namespace cabot_navigation2 {

CaBotPlanner::CaBotPlanner(): cost_(nullptr), mark_(nullptr), data_(nullptr), idx_(nullptr) {}

CaBotPlanner::~CaBotPlanner() {}

void CaBotPlanner::cleanup() {}

void CaBotPlanner::activate() {}

void CaBotPlanner::deactivate() {}

void CaBotPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                             std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();

  auto node = parent_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring Cabot Planner: %s", name_.c_str());

  declare_parameter_if_not_declared(node, name + ".path_width", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".path_width", options_.path_width);

  declare_parameter_if_not_declared(node, name + ".path_min_width", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".path_min_width", options_.path_min_width);

  declare_parameter_if_not_declared(node, name + ".path_adjusted_center", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".path_adjusted_center", options_.path_adjusted_center);

  declare_parameter_if_not_declared(node, name + ".path_adjusted_minimum_path_width", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".path_adjusted_minimum_path_width", options_.path_adjusted_minimum_path_width);

  declare_parameter_if_not_declared(node, name + ".safe_margin", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".safe_margin", options_.safe_margin);

  declare_parameter_if_not_declared(node, name + ".robot_radius", rclcpp::ParameterValue(0.45));
  node->get_parameter(name + ".robot_radius", options_.robot_radius);

  declare_parameter_if_not_declared(node, name + ".path_length_to_width_factor", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".path_length_to_width_factor", options_.path_length_to_width_factor);

  declare_parameter_if_not_declared(node, name + ".path_topic", rclcpp::ParameterValue("/path"));
  node->get_parameter(name + ".path_topic", path_topic_);

  declare_parameter_if_not_declared(node, name + ".cost_threshold", rclcpp::ParameterValue(254));
  node->get_parameter(name + ".cost_threshold", cost_threshold_);

  callback_handler_ =
      node->add_on_set_parameters_callback(std::bind(&CaBotPlanner::param_set_callback, this, std::placeholders::_1));

  rclcpp::QoS path_qos(10);
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      path_topic_, path_qos, std::bind(&CaBotPlanner::pathCallBack, this, std::placeholders::_1));
}

nav_msgs::msg::Path CaBotPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                             const geometry_msgs::msg::PoseStamped &goal) {
  if (navcog_path_ == nullptr) {
    RCLCPP_INFO(logger_, "navcog path is null");
    return nav_msgs::msg::Path();
  }

  RCLCPP_INFO(logger_, "navcog_path pose size %ld", navcog_path_->poses.size());
  nav_msgs::msg::Path path = normalizedPath(*navcog_path_);
  RCLCPP_INFO(logger_, "normalized path pose size %ld", path.poses.size());
  if (path.poses.empty()) {
    return nav_msgs::msg::Path();
  }
  for(unsigned long i = 0; i < path.poses.size(); i++){
    RCLCPP_INFO(logger_, "path[%3d]=(%.2f, %.2f)", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
  }

  /*
  estimatePathWidthAndAdjust(path, costmap_, options_);
  RCLCPP_INFO(logger_, "adjusted path pose size %ld", path.poses.size());
  for(unsigned long i = 0; i < path.poses.size(); i++){
    RCLCPP_INFO(logger_, "path[%3d]=(%.2f, %.2f)", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
  }
  */

  path = adjustedPathByStart(path, start);
  RCLCPP_INFO(logger_, "adjusted path by start size %ld", path.poses.size());
  RCLCPP_INFO(logger_, "start=(%.2f, %.2f)", start.pose.position.x, start.pose.position.y);
  for(unsigned long i = 0; i < path.poses.size(); i++){
    RCLCPP_INFO(logger_, "path[%3d]=(%.2f, %.2f)", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
  }

  RCLCPP_INFO(logger_, "cabot planner ---- start iteration", path.poses.size());
  setParam(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
        costmap_->getOriginX(), costmap_->getOriginY(), costmap_->getResolution(), DetourMode::RIGHT);
  setCost(costmap_->getCharMap());
  setPath(path);
  findObstacles();
  auto t0 = std::chrono::system_clock::now();
  int count = 0;
  while (rclcpp::ok()) {
    bool result = iterate();
    count++;
    if (result) {
      break;
    }
  }
  auto t1 = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  RCLCPP_INFO(logger_, "cabot planner ---- iteration count = %d: duration: %ldms", count, ms.count());

  auto plan = getPlan();
  RCLCPP_INFO(logger_, "cabot planner ---- plan size = %ld", plan.poses.size());
  return plan;
}

// prepare navcog path by topic
void CaBotPlanner::pathCallBack(nav_msgs::msg::Path::SharedPtr path) {
  navcog_path_ = path;
  RCLCPP_INFO(logger_, "received navcog path");
}

// private functions
rcl_interfaces::msg::SetParametersResult CaBotPlanner::param_set_callback(const std::vector<rclcpp::Parameter> params) {
  auto node = parent_.lock();

  RCLCPP_INFO(logger_, "CaBotPlanner::param_set_callback");
  auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

  for (auto &&param : params) {
    RCLCPP_INFO(logger_, "change param %s", param.get_name().c_str());
    if (!node->has_parameter(param.get_name())) {
      continue;
    }

    if (param.get_name() == name_ + ".path_adjusted_center") {
      options_.path_adjusted_center = param.as_double();
    }

    if (param.get_name() == name_ + ".path_adjusted_minimum_path_width_") {
      options_.path_adjusted_minimum_path_width = param.as_double();
    }
    if (param.get_name() == name_ + ".cost_threshold") {
      cost_threshold_ = param.as_int();
    }
  }
  results->successful = true;
  results->reason = "";
  return *results;
}

void CaBotPlanner::setParam(int width, int height, float origin_x, float origin_y, float resolution,
                            DetourMode detour) {
  RCLCPP_INFO(logger_, "setParam w=%d h=%d ox=%.2f oy=%.2f res=%.2f", width, height, origin_x, origin_y, resolution);
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
  if (mark_) {
    delete mark_;
  }
  mark_ = new unsigned char[width_ * height_];
}

bool CaBotPlanner::worldToMap(float wx, float wy, float &mx, float &my) {
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  mx = (wx - origin_x_) / resolution_;
  my = (wy - origin_y_) / resolution_;

  if (mx < width_ && my < height_) {
    return true;
  }
  return false;
}

void CaBotPlanner::mapToWorld(float mx, float my, float &wx, float &wy) {
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
}

int CaBotPlanner::getIndex(float x, float y) {
  if (x < 0 || y < 0 || width_ < x || height_ < y) {
    return -1;
  }
  return static_cast<int>(x) + static_cast<int>(y) * width_;
}

int CaBotPlanner::getIndexByPoint(Point &p) { return getIndex(p.x, p.y); }

void CaBotPlanner::setCost(unsigned char *cost) {
  unsigned char *p = cost_;
  unsigned char *p2 = mark_;
  for (int x = 0; x < width_; x++) {
    for (int y = 0; y < width_; y++) {
      *p++ = *cost++;
      *p2++ = 0;
    }
  }
}

void CaBotPlanner::setPath(nav_msgs::msg::Path path) {
  path_ = path;
  nodes_ = getNodesFromPath(path_);
}

nav_msgs::msg::Path CaBotPlanner::getPlan(void) {
  nav_msgs::msg::Path ret;
  ret.header.frame_id = "map";

  if (nodes_.size() == 0) {
    return ret;
  }

  auto mp0 = nodes_[0];
  for (long unsigned int i = 1; i < nodes_.size(); i++) {
    auto mp1 = nodes_[i];
    geometry_msgs::msg::PoseStamped wp;
    float mx, my;
    mapToWorld(mp0.x, mp0.y, mx, my);
    wp.pose.position.x = mx;
    wp.pose.position.y = my;
    
    float dy = mp1.y - mp0.y;
    float dx = mp1.x - mp0.x;
    float yaw = 0;
    if (dy != 0 && dx != 0) {
      yaw = std::atan2(mp1.y - mp0.y, mp1.x - mp0.x);
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    wp.pose.orientation.x = q.x();
    wp.pose.orientation.y = q.y();
    wp.pose.orientation.z = q.z();
    wp.pose.orientation.w = q.w();
    ret.poses.push_back(wp);

    mp0 = mp1;
  }
  return ret;
}

bool CaBotPlanner::iterate() {
  float scale = 0.1;
  float gravity_factor = 1.0;
  float link_spring_factor = 1.0;     // spring factor;
  float anchor_spring_factor = 0.01;  // spring factor;
  float complete_threshold = 0.02;

  std::vector<Node> newNodes;

  for (unsigned long i = 0; i < nodes_.size(); i++) {
    Node *n1 = &nodes_[i];
    Node newNode;
    newNode.x = n1->x;
    newNode.y = n1->y;
    newNodes.push_back(newNode);
  }

// Compute forces for each node.
// This section only depends on nodes_ and i, so it can be processed parallely
// OMP_NUM_THREADS environment variable can change the number of threads
// ~4 could be useful, too many threads can cause too much overhead
// usually, this section takes a few milli seconds
// #pragma omp parallel for
  for (unsigned long i = 1; i < nodes_.size(); i++) {
    Node *n0 = &nodes_[i - 1];
    Node *n1 = &nodes_[i];
    Node *newNode = &newNodes[i];

    // gravity term with obstacles that the path collide
    std::set<ObstacleGroup>::iterator ogit;
    for (ogit = groups_.begin(); ogit != groups_.end(); ++ogit) {
      if (detour_ == DetourMode::IGNORE) {
        continue;
      }
      float d = ogit->distance(*n1);
      d = std::max(0.0f, d - ogit->getSize(*n1));
      float yaw = 0;
      if (d < 0.5) {
        d = 0.5;
        if (detour_ == DetourMode::RIGHT) {
          yaw = (*n1 - *n0).yaw(-M_PI_2);
        } else {
          yaw = (*n1 - *n0).yaw(+M_PI_2);
        }
      } else {
        yaw = (*n1 - *ogit).yaw();
      }
      float m = gravity_factor / d / d * scale;
      newNode->move(yaw, m);
    }

    // gravity term with other obstacles
    std::vector<Obstacle> list = getObstaclesNearNode(*n1);
    std::vector<Obstacle>::iterator it;
    for (it = list.begin(); it != list.end(); ++it) {
      if (it->invalid) continue;
      // float d = std::min(0.0, it->distance(*n1) - it->size);
      float d = it->distance(*n1);
      d = std::max(0.0f, d - it->size);
      float yaw = 0;
      if (d < 0.5) {
        d = 0.5;
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
    if (d > 0.1) {
      float yaw = (n1->anchor - *n1).yaw();
      newNode->move(yaw, d * anchor_spring_factor * scale);
    }

    // spring term with adjusent nodes
    if (i < nodes_.size() - 1) {
      Node *n2 = &nodes_[i + 1];

      d = n0->distance(*n1);
      if (d > 0.01) {
        float yaw = (*n0 - *n1).yaw();
        newNode->move(yaw, d * link_spring_factor * scale);
      }
      // RCLCPP_INFO(logger_, "%d: %.2f %.2f ", i, d, yaw);
      d = n2->distance(*n1);
      if (d > 0.01) {
        float yaw = (*n2 - *n1).yaw();
        newNode->move(yaw, d * link_spring_factor * scale);
      }
      // RCLCPP_INFO(logger_, "%d: %.2f %.2f\n", i, d, yaw);
      // RCLCPP_INFO(logger_, "(%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)\n", n0->x,
      // n0->y, n1->x, n1->y, newNode.x, newNode.y, n2->x, n2->y);
    }
  }

  // check if it is converged
  bool complete_flag = true;
  for (unsigned long i = 0; i < nodes_.size(); i++) {
    float dx = newNodes[i].x - nodes_[i].x;
    float dy = newNodes[i].y - nodes_[i].y;
    float dist = std::hypot(dx, dy);
    if (dx > complete_threshold || dy > complete_threshold) {
      complete_flag = false;
    } else if (dist > 0.5) {
      newNodes[i].x = nodes_[i].x + dx / dist * 0.5;
      newNodes[i].y = nodes_[i].y + dy / dist * 0.5;
    }
    nodes_[i].x = newNodes[i].x;
    nodes_[i].y = newNodes[i].y;
  }

  if (complete_flag) {
    RCLCPP_INFO(logger_, "less than the threshold and completed\n");
    // if converged path collides with obstacle, change detoure mode
    for (unsigned long i = 0; i < nodes_.size(); i++) {
      int index = getIndexByPoint(nodes_[i]);
      if (cost_[index] > 127) {
        if (detour_ == DetourMode::RIGHT) {
          detour_ = DetourMode::LEFT;
          resetNodes();
          RCLCPP_INFO(logger_, "right side cannot be passsed\n");
          return false;
        } else if (detour_ == DetourMode::LEFT) {
          detour_ = DetourMode::IGNORE;
          resetNodes();
          RCLCPP_INFO(logger_, "left side also cannot be passsed\n");
          return false;
        } else if (detour_ == DetourMode::IGNORE) {
          RCLCPP_INFO(logger_, "ignore mode may collide with obstacles\n");
          return true;
        } else {
          RCLCPP_INFO(logger_, "unkown detour mode\n");
          return true;
        }
      }
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
    if (std::abs(total_yaw_diff) > M_PI * 7 / 4) {
      if (detour_ == DetourMode::RIGHT) {
        detour_ = DetourMode::LEFT;
        resetNodes();
        RCLCPP_INFO(logger_, "the path made a round: right\n");
        return false;
      } else if (detour_ == DetourMode::LEFT) {
        RCLCPP_INFO(logger_, "the path made a round: left\n");
        return true;
      } else if (detour_ == DetourMode::IGNORE) {
        RCLCPP_INFO(logger_, "the path made a round: ignore\n");
        return true;
      } else {
        RCLCPP_INFO(logger_, "the path made a round: unknown detour mode");
        return true;
      }
    }
  }

  // RCLCPP_INFO(logger_, "----------------------------------------------\n");
  return complete_flag;
}

// protected methods
void CaBotPlanner::resetNodes() {
  for (unsigned long i = 0; i < nodes_.size(); i++) {
    nodes_[i].reset();
  }
}

void CaBotPlanner::scanObstacleAt(ObstacleGroup &group, float mx, float my, unsigned int min_obstacle_cost) {
  int index = getIndex(mx, my);
  if (cost_[index] < min_obstacle_cost) {
    return;
  }
  if (mark_[index] != 0) {
    return;
  }
  group.add(Obstacle(mx, my, index, 5));
  mark_[index] = 255;
  if (mx + 1 < width_) {
    scanObstacleAt(group, mx + 1, my, min_obstacle_cost);
  }
  if (my + 1 < height_) {
    scanObstacleAt(group, mx, my + 1, min_obstacle_cost);
  }
  if (mx - 1 >= 0) {
    scanObstacleAt(group, mx - 1, my, min_obstacle_cost);
  }
  if (my - 1 >= 0) {
    scanObstacleAt(group, mx, my - 1, min_obstacle_cost);
  }
}

void CaBotPlanner::findObstacles() {
  // check obstacles only from current position to N meters forward
  // traverse the path
  int MIN_OBSTACLE_COST = 253;

  groups_.clear();
  obstacles_.clear();
  std::vector<Node>::iterator it;
  for (it = nodes_.begin(); it != nodes_.end(); it++) {
    int index = getIndexByPoint(*it);
    if (index < 0) {
      continue;
    }
    auto cost = cost_[index];
    // if the path is on an lethal cost (254) cell
    // extract all adjusent lethal cost cells
    if (cost >= MIN_OBSTACLE_COST && mark_[index] == 0) {
      ObstacleGroup group;
      scanObstacleAt(group, it->x, it->y, MIN_OBSTACLE_COST);
      group.complete();
      RCLCPP_INFO(logger_, "Group Obstacle %.2f %.2f %.2f %ld", group.x, group.y, group.size, group.obstacles_.size());
      groups_.insert(group);
    }
  }

  // find surrounding lethal cost except the obstacles the path is on
  std::vector<int> marks;
  for (it = nodes_.begin(); it != nodes_.end(); it++) {
    int index = getIndexByPoint(*it);
    if (index < 0) {
      continue;
    }
    auto cost = cost_[index];
    if (cost <= MIN_OBSTACLE_COST) {
      mark_[index] = 1;
      marks.push_back(index);
    }
  }
  RCLCPP_INFO(logger_, "marks size = %ld", marks.size());
  unsigned long i = 0;
  int MAX = 40;
  int max_size = width_ * height_;
  while (i < marks.size()) {
    int index = marks[i++];
    auto cost = cost_[index];
    if (cost > MIN_OBSTACLE_COST) {
      float x = index % width_;
      float y = index / width_;
      obstacles_.insert(Obstacle(x, y, index, 1.6));
      // RCLCPP_INFO(logger_, "%.2f %.2f\n", x, y);
    }
    unsigned char current = mark_[index];
    if (current >= MAX) continue;
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

  RCLCPP_INFO(logger_, "found %ld obstacles", obstacles_.size());
  unsigned long n = obstacles_.size();
  if (idx_) {
    delete idx_;
    idx_ = nullptr;
  }
  if (data_) {
    delete data_;
    data_ = nullptr;
  }
  if (n == 0) {
    return;
  }
  data_ = new cv::Mat(n, 2, CV_32FC1);
  olist_.clear();
  i = 0;
  std::set<Obstacle>::iterator oit;
  for (oit = obstacles_.begin(); oit != obstacles_.end(); ++oit) {
    olist_.push_back(*oit);
    data_->at<float>(i, 0) = oit->x;
    data_->at<float>(i, 1) = oit->y;
    i++;
  }
  RCLCPP_INFO(logger_, "making index %ld/%ld", i, n);
  idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  // RCLCPP_INFO(logger_, "obstacles = %ld\n", obstacles_.size());
}

std::vector<Obstacle> CaBotPlanner::getObstaclesNearNode(Node &node) {
  std::vector<Obstacle> list;
  if (!idx_ || !data_) {
    return list;
  }
  cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
  query.at<float>(0) = node.x;
  query.at<float>(1) = node.y;
  std::vector<int> indices;
  std::vector<float> dists;

  int m = idx_->radiusSearch(query, indices, dists, 100, 100);
    for (int i = 0; i < m; i++) {
    list.push_back(olist_[indices[i]]);
  }
  return list;
}

/*
 * generate map coordinate points
 */
std::vector<Node> CaBotPlanner::getNodesFromPath(nav_msgs::msg::Path path) {
  std::vector<Node> nodes;

  // push initial pose
  auto p0 = path.poses[0].pose.position;
  for (long unsigned int i = 1; i < path.poses.size(); i++) {
    auto p1 = path.poses[i].pose.position;

    auto dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
    int N = std::ceil(dist / 0.10);

    for (int j = 0; j <= N; j++) {
      // deal with the last pose
      if (j == N && i < path.poses.size() - 1) {
        continue;
      }
      float mx = 0, my = 0;
      worldToMap((p0.x * (N - j) + p1.x * j) / N, (p0.y * (N - j) + p1.y * j) / N, mx, my);
      nodes.push_back(Node(mx, my));
    }
    p0 = p1;
  }

  return nodes;
}

}  // namespace cabot_navigation2
