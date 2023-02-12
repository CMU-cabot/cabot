// Copyright (c) 2020  Carnegie Mellon University
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

#include <behaviortree_cpp_v3/condition_node.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class CheckPathCondition : public BT::ConditionNode
{
public:
  CheckPathCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    path_okay_(false),
    target_path_ready_(false)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string path_topic;
    if (!getInput("path_topic", path_topic)) {
      path_topic = "path";
    }
    rclcpp::QoS qos(10);
    qos.transient_local();
    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      path_topic, qos, std::bind(&CheckPathCondition::path_callback, this, std::placeholders::_1));

    std::string plan_topic;
    if (!getInput("plan_topic", plan_topic)) {
      plan_topic = "plan";
    }
    RCLCPP_INFO(node_->get_logger(), "prepareing plan publisher %s", plan_topic.c_str());
    plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(plan_topic, 10);

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an CheckPath");
  }

  CheckPathCondition() = delete;

  ~CheckPathCondition()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down CheckPathCondition BT node");
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    double MIN_DIST = 0.1;
    auto dist = [](geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2) {
        auto dx = p1.pose.position.x - p2.pose.position.x;
        auto dy = p1.pose.position.y - p2.pose.position.y;
        return sqrt(dx * dx + dy * dy);
      };
    auto yaw12 = [](geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2) {
        auto dx = p2.pose.position.x - p1.pose.position.x;
        auto dy = p2.pose.position.y - p1.pose.position.y;
        return atan2(dy, dx);
      };

    geometry_msgs::msg::PoseStamped prev;
    nav_msgs::msg::Path target_path;

    for (uint64_t i = 0; i < msg->poses.size() - 1; i++) {
      auto p1 = msg->poses[i];
      auto p2 = msg->poses[i + 1];

      target_path.poses.push_back(p1);
      while (dist(p1, p2) > MIN_DIST) {
        auto p3 = p1;
        auto yaw = yaw12(p1, p2);
        p3.pose.position.x += cos(yaw) * MIN_DIST;
        p3.pose.position.y += sin(yaw) * MIN_DIST;
        target_path.poses.push_back(p3);
        p1 = p3;
      }
    }
    correct_orientation(target_path);
    target_path_ = target_path;
    target_path_ready_ = true;
  }

  void smoothing_path(nav_msgs::msg::Path & path)
  {
    for (int j = 0; j < 5; j++) {
      for (uint64_t i = 0; i < path.poses.size() - 2; i++) {
        auto p0 = path.poses[i];
        auto p2 = path.poses[i + 2];
        path.poses[i + 1].pose.position.x = (p0.pose.position.x + p2.pose.position.x) / 2.0;
        path.poses[i + 1].pose.position.y = (p0.pose.position.y + p2.pose.position.y) / 2.0;
      }
    }
  }

  void correct_orientation(nav_msgs::msg::Path & path)
  {
    for (uint64_t i = 0; i < path.poses.size() - 1; i++) {
      double yaw = atan2(
        path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y,
        path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
      tf2::Quaternion quat;
      quat.setRPY(0, 0, yaw);
      path.poses[i].pose.orientation = tf2::toMsg(quat);
    }
    path.poses[path.poses.size() - 1].pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
  }

  void check_path()
  {
    path_okay_ = false;

    if (!getInput("path", path_)) {
      RCLCPP_ERROR(node_->get_logger(), "path is missing");
      return;
    }

    if (path_.poses.size() == 0 || target_path_.poses.size() == 0) {
      return;
    }

    smoothing_path(path_);
    correct_orientation(path_);
    // correct_orientation(target_path_);

    double average_threshold = 1.5;
    getInput("average_threshold", average_threshold);
    double maximum_threshold = 3.0;
    getInput("maximum_threshold", maximum_threshold);

    auto path_target_similar = similar_enough(path_, target_path_, average_threshold, maximum_threshold);
    auto path_current_similar = false;

    if (getInput("current", current_)) {
      path_current_similar = similar_enough(path_, current_, 0.2, 0.25);
    }

    path_okay_ = path_target_similar && !path_current_similar;
  }

  bool similar_enough(nav_msgs::msg::Path path, nav_msgs::msg::Path target_path, double average_threshold, double maximum_threshold)
  {
    // find the closest point
    auto start = path.poses[0];
    int start_i = 0;
    double min_dist = 1000;
    for (uint64_t i = 0; i < target_path.poses.size(); i++) {
      auto p = target_path.poses[i];
      auto dist = sqrt(
        pow(p.pose.position.x - start.pose.position.x, 2) +
        pow(p.pose.position.y - start.pose.position.y, 2));
      if (dist < min_dist) {
        min_dist = dist;
        start_i = i;
      }
    }

    int n = target_path.poses.size() - start_i + 1;
    int m = path.poses.size() + 1;

    // calculate minimum cost
    double * cost = static_cast<double *>(malloc(sizeof(double) * n * m));
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < m; j++) {
        cost[i * m + j] = std::numeric_limits<double>::max();
      }
    }
    cost[0] = 0;
    tf2::Quaternion q1, q2;
    for (int i = 1; i < n; i++) {
      for (int j = 1; j < m; j++) {
        auto p1 = target_path.poses[start_i + i - 1];
        auto p2 = path.poses[j - 1];

        double dist = sqrt(
          pow(p1.pose.position.x - p2.pose.position.x, 2) +
          pow(p1.pose.position.y - p2.pose.position.y, 2));
        tf2::fromMsg(p1.pose.orientation, q1);
        tf2::fromMsg(p2.pose.orientation, q2);
        if (abs(tf2::angleShortestPath(q1, q2)) > M_PI / 4 * 3) {
          dist += 1000;
        }
        cost[i * m + j] = dist + std::min(std::min(cost[(i - 1) * m + j], cost[i * m + j - 1]), cost[(i - 1) * m + j - 1]);
      }
    }

    // count only distant points
    int count = 0;
    double total_dist = 0;
    double max_dist = 0;
    for (int i = 0, j = 0; i < n && j < m; ) {
      auto current = cost[i * m + j];
      auto min = std::numeric_limits<double>::max();
      if (i < n - 1) {
        min = std::min(min, cost[(i + 1) * m + j]);
      }
      if (j < m - 1) {
        min = std::min(min, cost[i * m + j + 1]);
      }
      if (i < n - 1 && j < m - 1) {
        min = std::min(min, cost[(i + 1) * m + j + 1]);
      }

      auto dist = min - current;
      // RCLCPP_INFO(node_->get_logger(), "[%d/%d, %d/%d] current %.2f min %.2f dist %.2f total %.2f count = %d",
      // i, n, j, m, current, min, dist, total_dist, count);
      if (dist > 0.1) {
        count++;
        total_dist += dist;
        if (max_dist < dist) {
          max_dist = dist;
        }
      }
      if (i < n - 1 && j < m - 1 && min == cost[(i + 1) * m + j + 1]) {
        i++;
        j++;
      } else if (i < n - 1 && min == cost[(i + 1) * m + j]) {
        i++;
      } else if (j < m - 1 && min == cost[i * m + j + 1]) {
        j++;
      }

      if (i == n - 1 && j == m - 1) {
        break;
      }
    }

    double ave_dist = (count > 0) ? total_dist / count : 0;

    auto okay = (ave_dist < average_threshold) && (max_dist < maximum_threshold);

    RCLCPP_INFO(
      node_->get_logger(), "check_path[%d](n %d(%d), m %d) total_dist %.2f, count %d, ave_dist %.2f, average_threshold %.2f, max_dist %.2f, maximum_threshold %.2f",
      okay, n, start_i, m, total_dist, count, ave_dist, average_threshold, max_dist, maximum_threshold);

    if (false) {
      // log replaced path
      for (auto p : target_path.poses) {
        RCLCPP_INFO(
          node_->get_logger(), "target_path.poses %8.2f %8.2f %8.2f",
          p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation));
      }
      for (auto p : path.poses) {
        RCLCPP_INFO(
          node_->get_logger(), "path.poses        %8.2f %8.2f %8.2f",
          p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation));
      }
    }

    free(cost);
    return okay;
  }

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE || target_path_ready_ == false) {
      rclcpp::spin_some(node_);
      return BT::NodeStatus::RUNNING;
    }

    check_path();

    if (path_okay_) {
      RCLCPP_INFO(node_->get_logger(), "Path is okay");
      setOutput("path_out", path_);
      plan_pub_->publish(path_);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "Path is not okays");
    return BT::NodeStatus::FAILURE;
  }

  void logStuck(const std::string & msg) const
  {
    static std::string prev_msg;

    if (msg == prev_msg) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    prev_msg = msg;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<nav_msgs::msg::Path>("current", "The current path"),
      BT::InputPort<nav_msgs::msg::Path>("path", "The path to be checked"),
      BT::InputPort<std::string>("path_topic", "topic where navcog path published"),
      BT::InputPort<std::string>("plan_topic", "topic where path to be published"),
      BT::InputPort<double>("average_threshold", "path ignoreing threshold"),
      BT::InputPort<double>("maximum_threshold", "path ignoreing threshold"),
      BT::OutputPort<nav_msgs::msg::Path>("path_out", "The path if the input path is okay"),
    };
  }

private:
  std::atomic<bool> path_okay_;
  std::atomic<bool> target_path_ready_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  nav_msgs::msg::Path target_path_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path current_;


  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::CheckPathCondition>("CheckPath");
}
