/*******************************************************************************
 * Copyright (c) 2023  Carnegie Mellon University
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define NUM_MODES (2)
std::string MODE_NAMES[2] = {"init", "track"};

typedef struct FloorData {
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_points_sub;
} FloorData;

class MultiFloorTopicProxy : public rclcpp::Node
{
 public:
  MultiFloorTopicProxy()
      : Node("multi_floor_topic_proxy"),
        current_floor(0),
        current_area(0),
        current_mode(0)
  {
    std::string map_config_file = this->declare_parameter("map_config_file", "");
    bool verbose = this->declare_parameter("verbose", false);

    YAML::Node config = YAML::LoadFile(map_config_file);
    if (!config["map_list"]) {
      RCLCPP_INFO(this->get_logger(), "map_list not found in YAML file");
      return;
    }

    std::string imu_topic_name = this->get_node_topics_interface()->resolve_topic_name("imu");
    std::string points2_topic_name = this->get_node_topics_interface()->resolve_topic_name("points2");
    std::string odom_topic_name = this->get_node_topics_interface()->resolve_topic_name("odom");

    auto map_list = config["map_list"];
    for (YAML::const_iterator it = map_list.begin(); it != map_list.end(); ++it) {
      YAML::Node map_dict = *it;
      
      std::string node_id = map_dict["node_id"].as<std::string>();
      
      int floor = map_dict["floor"].as<int>();
      int area = map_dict["area"].as<int>();
      for (int mode = 0; mode < NUM_MODES; mode ++) {
        auto mode_str = MODE_NAMES[mode];

        std::string key = getKey(floor, area, mode);
        
        FloorData floordata = {nullptr, nullptr, nullptr};
        RCLCPP_INFO(this->get_logger(), "floor = %d, area = %d, mode=%d, key=%s", floor, area, mode, key.c_str());
        
        floordata.imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(node_id+"/"+mode_str+imu_topic_name, 1000);
        floordata.points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_id+"/"+mode_str+points2_topic_name, 100);
        floordata.odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(node_id+"/"+mode_str+odom_topic_name, 100);
        floordata.scan_matched_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(node_id+"/"+mode_str+"/scan_matched_points2", 10,
                                                                                                     std::bind(&MultiFloorTopicProxy::scan_matched_points2_callback, this, _1));
        floor_map[key] = floordata;
      }
    }
    
    auto latched_qos = rclcpp::QoS(10).transient_local();
    current_floor_sub = this->create_subscription<std_msgs::msg::Int64>("current_floor", latched_qos, std::bind(&MultiFloorTopicProxy::current_floor_callback, this, _1));
    current_area_sub = this->create_subscription<std_msgs::msg::Int64>("current_area", latched_qos, std::bind(&MultiFloorTopicProxy::current_area_callback, this, _1));
    current_mode_sub = this->create_subscription<std_msgs::msg::Int64>("current_mode", latched_qos, std::bind(&MultiFloorTopicProxy::current_mode_callback, this, _1));

    rclcpp::SensorDataQoS sensor_qos;
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", sensor_qos, std::bind(&MultiFloorTopicProxy::imu_callback, this, _1));
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("points2", sensor_qos, std::bind(&MultiFloorTopicProxy::points_callback, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", sensor_qos, std::bind(&MultiFloorTopicProxy::odom_callback, this, _1));
    scan_matched_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_matched_points2", 10);
  }

  std::string getKey(const int & floor, const int & area, const int & mode) const {
    return std::to_string(floor) + "-" + std::to_string(area) + "-" + std::to_string(mode);
  }

  void scan_matched_points2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    scan_matched_points_pub->publish(*msg);
  }

  void current_floor_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    this->current_floor = msg->data;
    RCLCPP_INFO(this->get_logger(), "floor=%d, area=%d, mode=%d", current_floor, current_area, current_mode);
  }

  void current_area_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    this->current_area = msg->data;
    RCLCPP_INFO(this->get_logger(), "floor=%d, area=%d, mode=%d", current_floor, current_area, current_mode);
  }

  void current_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    this->current_mode = msg->data;
    RCLCPP_INFO(this->get_logger(), "floor=%d, area=%d, mode=%d", current_floor, current_area, current_mode);
  }
  

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::string key = getKey(this->current_floor, this->current_area, this->current_mode);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "imu called");
    auto search = floor_map.find(key);
    if (search == floor_map.end()) {
      return;
    }

    double norm_q_tolerance = 0.1;
    double norm_acc_threshold = 0.1;

    auto acc = msg->linear_acceleration;
    auto q = msg->orientation;
    auto norm_acc = sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
    auto norm_q = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (norm_acc_threshold > norm_acc || std::abs(norm_q-1.0) > norm_q_tolerance) {
      RCLCPP_INFO(this->get_logger(),
                  "imu input is invalid. (linear_acceleration=(%.5f,%.5f,%.5f), orientation=(%.5f,%.5f,%.5f,%.5f))",
                  acc.x, acc.y, acc.z, q.x, q.y, q.z, q.w);
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "imu publish");
    search->second.imu_pub->publish(*msg);
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::string key = getKey(this->current_floor, this->current_area, this->current_mode);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "points called");
    auto search = floor_map.find(key);
    if (search == floor_map.end()) {
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "points publish");
    search->second.points_pub->publish(*msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::string key = getKey(this->current_floor, this->current_area, this->current_mode);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "odom called");
    auto search = floor_map.find(key);
    if (search == floor_map.end()) {
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "odom publish");
    search->second.odom_pub->publish(*msg);
  }
  
 private:
  std::unordered_map<std::string, FloorData> floor_map;

  int current_floor;
  int current_area;
  int current_mode;
  
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr current_floor_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr current_area_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr current_mode_sub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_points_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiFloorTopicProxy>());
  rclcpp::shutdown();
  return 0;
}
