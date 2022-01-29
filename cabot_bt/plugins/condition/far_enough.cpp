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

#include <string>
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/person_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

using namespace std::chrono_literals;

namespace cabot_bt
{

      class FarEnoughCondition : public BT::ConditionNode
      {
      public:
            FarEnoughCondition(
                const std::string &condition_name,
                const BT::NodeConfiguration &conf)
                : BT::ConditionNode(condition_name, conf),
                  are_far_enough_(false)
            {
                  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
                  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

                  std::string odom_topic;
                  if (!getInput("odom_topic", odom_topic))
                  {
                        RCLCPP_ERROR(node_->get_logger(), "odom_topic missing");
                        throw std::runtime_error("odom_topic missing");
                        return;
                  }
                  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                      odom_topic,
                      rclcpp::SystemDefaultsQoS(),
                      std::bind(&FarEnoughCondition::onOdomReceived, this, std::placeholders::_1));

                  RCLCPP_DEBUG(node_->get_logger(), "Initialized an FarEnough");
            }

            FarEnoughCondition() = delete;

            ~FarEnoughCondition()
            {
                  RCLCPP_DEBUG(node_->get_logger(), "Shutting down FarEnoughCondition BT node");
            }

            void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
            {
                  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry");
                  last_odom_ = msg;
            }

            void updateStates()
            {
                  std::string global_frame;
                  std::string robot_frame;
                  people_msgs::msg::PersonStamped person;
                  double max_acc;
                  double delay;
                  double min_threshold;
                  double max_threshold;
                  double V = 0;

                  if (!last_odom_)
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "odometry is missing, assumes the robot stops");
                  }
                  else
                  {
                        V = last_odom_->twist.twist.linear.x;
                  }
                  if (!getInput("max_acc", max_acc))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "max_acc is missing, assumes it is 0.5");
                        max_acc = 0.5;
                  }
                  if (!getInput("delay", delay))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "delay is missing, assumes it is 0.2");
                        delay = 0.2;
                  }
                  if (!getInput("min_threshold", min_threshold))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "min_threshold is missing, assumes it is 2.0");
                        min_threshold = 2.0;
                  }
                  if (!getInput("max_threshold", max_threshold))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "max_threshold is missing, assumes it is 4.0");
                        max_threshold = 4.0;
                  }
                  if (!getInput("global_frame", global_frame))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "global_frame missing");
                        are_far_enough_ = false;
                        return;
                  }
                  if (!getInput("robot_frame", robot_frame))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "robot_frame missing");
                        are_far_enough_ = false;
                        return;
                  }
                  if (!getInput("person", person))
                  {
                        RCLCPP_WARN_ONCE(node_->get_logger(), "person is missing");
                        are_far_enough_ = false;
                        return;
                  }

                  // transform person pose to global
                  geometry_msgs::msg::TransformStamped transform_person;
                  try
                  {
                        RCLCPP_DEBUG(node_->get_logger(), "lookupTransform %s->%s", person.header.frame_id.c_str(), global_frame.c_str());
                        transform_person = tf_buffer_->lookupTransform(global_frame, person.header.frame_id, rclcpp::Time(0));
                  }
                  catch (tf2::TransformException & ex)
                  {
                        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
                        return;
                  }

                  tf2::Transform tf2_transform_person;
                  tf2::convert(transform_person.transform, tf2_transform_person);

                  tf2::Vector3 person_position(person.person.position.x, person.person.position.y, 0);
                  tf2::Vector3 person_position_global;
                  person_position_global = tf2_transform_person * person_position;

                  // transform robot pose to global
                  RCLCPP_DEBUG(node_->get_logger(), "getCurrentPose %s %s", global_frame.c_str(), robot_frame.c_str());
                  geometry_msgs::msg::PoseStamped pose;
                  nav2_util::getCurrentPose(pose, *tf_buffer_, global_frame, robot_frame, 0.1);

                  // check if person is far enough
                  RCLCPP_DEBUG(node_->get_logger(), "current pose %.2f %.2f", pose.pose.position.x, pose.pose.position.y);
                  RCLCPP_DEBUG(node_->get_logger(), "person pose %.2f %.2f", person.person.position.x, person.person.position.y);
                  double dx = pose.pose.position.x - person_position_global.x();
                  double dy = pose.pose.position.y - person_position_global.y();
                  double dist = sqrt(dx * dx + dy * dy);
                  double BD = (V * V + 2 * max_acc * V * delay) / (2 * max_acc);

                  are_far_enough_ = (dist > min_threshold + BD) && (dist < max_threshold + BD);
                  RCLCPP_INFO(node_->get_logger(), "distance = %.2f, BD = %.2f, min-max = %.2f-%.2f",
                              dist, BD, min_threshold, max_threshold);
            }

            BT::NodeStatus tick() override
            {
                  updateStates();
                  rclcpp::spin_some(node_);
                  if (are_far_enough_)
                  {
                        return BT::NodeStatus::SUCCESS;
                  }
                  return BT::NodeStatus::FAILURE;
            }

            void logStuck(const std::string &msg) const
            {
                  static std::string prev_msg;

                  if (msg == prev_msg)
                  {
                        return;
                  }

                  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
                  prev_msg = msg;
            }

            static BT::PortsList providedPorts()
            {
                  return BT::PortsList{
                      BT::InputPort<std::string>("odom_topic", "Odometry topic to subscribe"),
                      BT::InputPort<double>("max_acc", "Maximum acceleration of the robot"),
                      BT::InputPort<double>("delay", "Delay of the robot control"),
                      BT::InputPort<std::string>("global_frame", "Global frame"),
                      BT::InputPort<std::string>("robot_frame", "Robot frame"),
                      BT::InputPort<people_msgs::msg::PersonStamped>("person", "person on the path"),
                      BT::InputPort<double>("min_threshold", "Min distance threshold"),
                      BT::InputPort<double>("max_threshold", "Max distance threshold"),
                  };
            }

      private:
            // The node that will be used for any ROS operations
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

            std::atomic<bool> are_far_enough_;

            // Listen to odometry
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            nav_msgs::msg::Odometry::SharedPtr last_odom_;
      };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
      factory.registerNodeType<cabot_bt::FarEnoughCondition>("FarEnough");
}
