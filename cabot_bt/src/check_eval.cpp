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

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "dwb_msgs/msg/local_plan_evaluation.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

bool sort_func(dwb_msgs::msg::TrajectoryScore s1, dwb_msgs::msg::TrajectoryScore s2) {
  //return s1.traj.velocity.theta < s2.traj.velocity.theta;
  if (s1.total < 0 && s2.total < 0) {
    return false;
  }
  if (s1.total < 0) {
    return false;
  }
  if (s2.total < 0) {
    return true;
  }
  return s1.total < s2.total;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("_");

  node->declare_parameter("bagfile", rclcpp::ParameterValue(std::string("")));
    
  std::string bag_file ="";
  if (node->get_parameter<std::string>("bagfile", bag_file)) {
    RCLCPP_INFO(node->get_logger(), "%s", bag_file.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "no bag param");
    return 0;
  }

  int items = 0;
  node->declare_parameter("items", rclcpp::ParameterValue(5));
  node->get_parameter<int>("items", items);
  RCLCPP_INFO(node->get_logger(), "print %d items", items);
    
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options{};
    
  storage_options.uri = bag_file;
  storage_options.storage_id = "sqlite3";
    
  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);
    
  auto topics = reader.get_all_topics_and_types();
  std::map<std::string, std::string> topic_type_map;
    
  // about metadata
  for (auto t:topics){
    std::cout << "meta name: " << t.name << std::endl;
    std::cout << "meta type: " << t.type << std::endl;
    std::cout << "meta serialization_format: " << t.serialization_format << std::endl;

    topic_type_map.insert({t.name, t.type});
  }


  rosbag2_cpp::SerializationFormatConverterFactory factory;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
  cdr_deserializer_ = factory.load_deserializer("cdr");
    
  // read and deserialize "serialized data"

  double total;
  double cmd;
  double vel;

  while (reader.has_next() && rclcpp::ok()){
      
    // serialized data
    auto serialized_message = reader.read_next();

    std::string type = topic_type_map[serialized_message->topic_name];
    
    rosbag2_cpp::ConverterTypeSupport type_support;
    type_support.type_support_library = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    type_support.rmw_type_support = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", type_support.type_support_library);

    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = rcutils_get_default_allocator();

    
    if (type == "dwb_msgs/msg/LocalPlanEvaluation") {
      dwb_msgs::msg::LocalPlanEvaluation msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      auto twist = msg.twists[msg.best_index];
      cmd = twist.traj.velocity.x;
      total = twist.total;
      
      int index = msg.best_index;
      printf("[%16d.%03d]\n", msg.header.stamp.sec, msg.header.stamp.nanosec/1000000);
      printf("[%4d] total=%10.4f (%4.2f, %4.2f)\n", index, twist.total, twist.traj.velocity.x, twist.traj.velocity.theta);
      for(auto score: twist.scores) {
	printf("%20s: %10.4f * %10.4f = %10.4f\n", score.name.c_str(), score.raw_score, score.scale, score.raw_score * score.scale);
      }

      sort(msg.twists.begin(), msg.twists.end(), sort_func);
      index = 0;
      for(auto twist: msg.twists) {
	printf("[%4d] total=%10.4f (%4.4f, %4.4f)\n", index, twist.total, twist.traj.velocity.x, twist.traj.velocity.theta);
	index++;
	for(auto score: twist.scores) {
	  printf("%20s: %10.4f * %10.4f = %10.4f\n", score.name.c_str(), score.raw_score, score.scale, score.raw_score * score.scale);
	}
	if (index > items) {
	  break;
	}
      }
    }
    if (type == "geometry_msgs/msg/Twist") {
      geometry_msgs::msg::Twist msg;
      ros_message->message = &msg;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      //std::cout << "cmd_vel (" << msg.linear.x << ", " << msg.angular.z << ")" << std::endl;
      vel = msg.linear.x;
    }


    //printf("[%8.4f] %8.4f, %8.4f\n", total, cmd, vel);
    
    // deserialization and conversion to ros message
    // ros message data

  }

  return 0;
}
