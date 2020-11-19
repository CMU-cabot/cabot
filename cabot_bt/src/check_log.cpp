// Copyright (c) 2020 Carnegie Mellon University
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
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_transport/storage_options.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("_");

  node->declare_parameter("bagfile", rclcpp::ParameterValue(std::string("")));
    
  std::string bag_file ="";
  if (node->get_parameter<std::string>("bagfile", bag_file)) {
    RCLCPP_INFO(node->get_logger(), bag_file);
  } else {
    RCLCPP_INFO(node->get_logger(), "no bag param");
    return 0;
  }
    
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_transport::StorageOptions storage_options{};
    
  storage_options.uri = bag_file;
  storage_options.storage_id = "sqlite3";
    
  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);
    
  auto topics = reader.get_all_topics_and_types();
    
  // about metadata
  for (auto t:topics){
    std::cout << "meta name: " << t.name << std::endl;
    std::cout << "meta type: " << t.type << std::endl;
    std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
  }

  int index = 0;

  std::string type = "nav2_msgs/msg/BehaviorTreeLog";
    
  rosbag2_cpp::ConverterTypeSupport type_support;
  type_support.type_support_library = rosbag2_cpp::get_typesupport_library(
    type, "rosidl_typesupport_cpp");
  type_support.rmw_type_support = rosbag2_cpp::get_typesupport_handle(
    type, "rosidl_typesupport_cpp",
    type_support.type_support_library);
      
  nav2_msgs::msg::BehaviorTreeLog msg;
  auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
  ros_message->time_stamp = 0;
  ros_message->message = nullptr;
  ros_message->allocator = rcutils_get_default_allocator();
  ros_message->message = &msg;
    
  rosbag2_cpp::SerializationFormatConverterFactory factory;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
  cdr_deserializer_ = factory.load_deserializer("cdr");
    
  // read and deserialize "serialized data"
  while (reader.has_next() && rclcpp::ok()){
    // serialized data
    auto serialized_message = reader.read_next();
    
    // deserialization and conversion to ros message
    try {
      
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      
    } catch (std::exception &error) {
      std::cout << error.what() << std::endl;
    }
    // ros message data
    
    int j = 0; 
    for(auto event:msg.event_log) {
      time_t t = event.timestamp.sec;
      struct tm *ts = localtime(&t);
      char buf[80];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ts);
      
      printf("%20s(%12d.%-8d)\t%30s\t%10s\t%10s\n", buf, event.timestamp.sec,
	     event.timestamp.nanosec, event.node_name.c_str(), event.previous_status.c_str(), event.current_status.c_str());
      index++;
      j++;
    }
  }

  return 0;
}
