// Copyright (c) 2023  Carnegie Mellon University
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

#include "../src/stop_reasoner.hpp"

using namespace cabot_ui;

#define ODOM_TOPIC "/cabot/odom_raw"
#define EVENT_TOPIC "/cabot/event"
#define RECEIVED_GLOBAL_PLAN "/received_global_plan"
#define CMD_VEL_TOPIC "/cmd_vel"
#define PEOPLE_SPEED_TOPIC "/cabot/people_speed"
#define TF_SPEED_TOPIC "/cabot/tf_speed"
#define TOUCH_SPEED_TOPIC "/cabot/touch_speed_switched"
#define LOCAL_PREFIX "/local"
#define REPLAN_REASON_TOPIC "/replan_reason"
#define CURRENT_FRAME_TOPIC "/current_frame"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_stop_reasoner_node");

  std::string bag_file = node->declare_parameter("bagfile", rclcpp::ParameterValue(std::string(""))).get<std::string>();
  double start = node->declare_parameter("start", rclcpp::ParameterValue(0.0)).get<double>();
  double duration = node->declare_parameter("duration", rclcpp::ParameterValue(99999999.0)).get<double>();

  if (bag_file == "") {
    RCLCPP_ERROR(node->get_logger(), "need to specify bagfile param");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "read from %s", bag_file.c_str());

  rosbag2_compression::SequentialCompressionReader reader;
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = bag_file;

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);

  auto topics = reader.get_all_topics_and_types();
  std::map<std::string, std::string> topic_type_map;
  for (auto t : topics) {
    topic_type_map.insert({t.name, t.type});
  }

  std::shared_ptr<StopReasoner> reasoner = std::make_shared<StopReasoner>(node);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {
    ODOM_TOPIC,
    EVENT_TOPIC,
    RECEIVED_GLOBAL_PLAN,
    CMD_VEL_TOPIC,
    PEOPLE_SPEED_TOPIC,
    TF_SPEED_TOPIC,
    TOUCH_SPEED_TOPIC,
    REPLAN_REASON_TOPIC,
    CURRENT_FRAME_TOPIC,
  };
  reader.set_filter(filter);

  std::shared_ptr<StopReasonFilter> stop_reason_filter =
    std::make_shared<StopReasonFilter>(
    std::vector<StopReason>(
  {
    StopReason::NO_NAVIGATION,
    StopReason::NOT_STOPPED,
    StopReason::NO_TOUCH,
    StopReason::STOPPED_BUT_UNDER_THRESHOLD
  }));

  rosbag2_cpp::SerializationFormatConverterFactory factory;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
  cdr_deserializer_ = factory.load_deserializer("cdr");

  StopReason prev_code = StopReason::NONE;

  int64_t start_time = 0;

  while (reader.has_next() && rclcpp::ok()) {
    auto serialized_message = reader.read_next();
    auto time_stamp = serialized_message->time_stamp;
    if (start_time == 0) {
      start_time = time_stamp;
    }
    auto time_from_start = (time_stamp - start_time) / 1e9f;
    if (time_from_start < start ||
      start + duration < time_from_start)
    {
      continue;
    }
    int64_t sec = RCUTILS_NS_TO_S(time_stamp);
    int64_t nsec = time_stamp - RCUTILS_S_TO_NS(sec);
    reasoner->update_time(rclcpp::Time(sec, nsec, RCL_SYSTEM_TIME));

    auto topic_name = serialized_message->topic_name;
    std::string type = topic_type_map[topic_name];
    rosbag2_cpp::ConverterTypeSupport type_support;
    type_support.type_support_library = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    type_support.rmw_type_support = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", type_support.type_support_library);

    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = rcutils_get_default_allocator();

    if (topic_name == ODOM_TOPIC) {
      nav_msgs::msg::Odometry msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_odom(msg);
    } else if (topic_name == EVENT_TOPIC) {
      std_msgs::msg::String msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_event(msg);
    } else if (topic_name == RECEIVED_GLOBAL_PLAN) {
      nav_msgs::msg::Path msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_global_plan(msg);
    } else if (topic_name == CMD_VEL_TOPIC) {
      geometry_msgs::msg::Twist msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_cmd_vel(msg);
    } else if (topic_name == PEOPLE_SPEED_TOPIC) {
      std_msgs::msg::Float32 msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_people_speed(msg);
    } else if (topic_name == TF_SPEED_TOPIC) {
      // noop
    } else if (topic_name == TOUCH_SPEED_TOPIC) {
      std_msgs::msg::Float32 msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_touch_speed(msg);
    } else if (topic_name == REPLAN_REASON_TOPIC) {
      people_msgs::msg::Person msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_replan_reason(msg);
    } else if (topic_name == CURRENT_FRAME_TOPIC) {
      std_msgs::msg::String msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      reasoner->input_current_frame(msg);
    }

    auto [duration, code] = reasoner->update();
    if (prev_code != code) {
      prev_code = code;
      RCLCPP_DEBUG(
        node->get_logger(), "%.2f(%.2f), %s, %.2f",
        sec + nsec / 1000000000.0, time_from_start,
        StopReasonUtil::toStr(code).c_str(), duration);
    }
    stop_reason_filter->update(duration, code);
    std::tie(duration, code) = stop_reason_filter->summary();
    stop_reason_filter->conclude();

    if (code != StopReason::NONE) {
      RCLCPP_INFO(
        node->get_logger(), "### %.2f(%.2f), %s, %.2f, %s",
        sec + nsec / 1000000000.0,
        time_from_start,
        StopReasonUtil::toStr(code).c_str(), duration,
        reasoner->is_navigating() ? "NAVIGATING" : "NOT NAVIGATING");
    }
  }

  return 0;
}
