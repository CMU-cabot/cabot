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

#include <memory.h>
#include <math.h>
#include "yaml-cpp/yaml.h"
#include "cabot_navigation2/cabot_planner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>

using namespace std::chrono_literals;
namespace fs = boost::filesystem;

namespace cabot_planner {
class Test : public rclcpp::Node {
 public:
  Test(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Test() {}
  void run_test();

 private:
  nav_msgs::msg::Path getPath();

  std::unique_ptr<Planner> planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path plan_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  std::unique_ptr<nav2_util::NodeThread> thread_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D *costmap_;
};
}  // namespace cabot_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_planner::Test>();
  node->run_test();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}

namespace cabot_planner {

template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

nav_msgs::msg::Path Test::getPath(){
  nav_msgs::msg::Path path;

  float wsx = -4.0;
  float wsy = -4.0;
  float wgx = 4.0;
  float wgy = 3.0;

  geometry_msgs::msg::PoseStamped pose0;
  pose0.pose.position.x = wsx;
  pose0.pose.position.y = wsy;
  path.poses.push_back(pose0);
  geometry_msgs::msg::PoseStamped pose1;
  pose1.pose.position.x = 4.0;
  pose1.pose.position.y = -4.0;
  path.poses.push_back(pose1);
  geometry_msgs::msg::PoseStamped pose2;
  pose2.pose.position.x = 4.0;
  pose2.pose.position.y = -1.0;
  path.poses.push_back(pose2);
  geometry_msgs::msg::PoseStamped pose3;
  pose3.pose.position.x = 0;
  pose3.pose.position.y = -1;
  path.poses.push_back(pose3);
  geometry_msgs::msg::PoseStamped pose4;
  pose4.pose.position.x = -0.5;
  pose4.pose.position.y = 3;
  path.poses.push_back(pose4);
  geometry_msgs::msg::PoseStamped pose5;
  pose5.pose.position.x = wgx;
  pose5.pose.position.y = wgy;
  path.poses.push_back(pose5);
  path.header.frame_id = "map";

  return path;
}

Test::Test(const rclcpp::NodeOptions &options)
    : rclcpp::Node("cabot_planner", "", options) {

  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
  timer_ = create_wall_timer(1s, [this]() -> void {
    map_publisher_->publish(map_);
    path_publisher_->publish(path_);
  });
  timer2_ = create_wall_timer(
      0.033s, [this]() -> void { plan_publisher_->publish(plan_); });

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{get_namespace()}, "global_costmap");
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  rclcpp_lifecycle::State state;
  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();
  costmap_ros_->on_activate(state);  
}

void Test::run_test() {
  fs::path base_path =
      ament_index_cpp::get_package_share_directory("cabot_navigation2");
  base_path /= "test";
  fs::path yaml_path = base_path / "test-cases.yaml";

  YAML::Node doc = YAML::LoadFile(yaml_path.string());
  const YAML::Node &tests = doc["tests"];

  planner_ = std::make_unique<Planner>();

  for(unsigned long i = 0; i < tests.size(); i++) {
    const YAML::Node &test = tests[i];
    auto label = yaml_get_value<std::string>(test, "label");
    auto map = yaml_get_value<std::string>(test, "map");
    auto path = yaml_get_value<std::vector<float>>(test, "path");
    auto detour = yaml_get_value<std::string>(test, "detour");
    auto skip = yaml_get_value<bool>(test, "skip");
    if (skip) continue;

    nav_msgs::msg::Path path_;
    for(unsigned long j = 0; j < path.size(); j+=2) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = path[j];
      pose.pose.position.y = path[j+1];
      path_.poses.push_back(pose);
    }

    fs::path map_path = base_path / map;
    nav2_map_server::LoadParameters yaml;
    if (boost::filesystem::exists(map_path)) {
      yaml = nav2_map_server::loadMapYaml(map_path.string());
      nav2_map_server::loadMapFromFile(yaml, map_);
    } else {
      printf("file not found\n");
    }

    DetourMode mode = DetourMode::RIGHT;
    if (detour == "left") {
      mode = DetourMode::LEFT;
    }

    planner_->setParam(map_.info.width, map_.info.height,
                       map_.info.origin.position.x, map_.info.origin.position.y,
                       map_.info.resolution, mode);

    planner_->setPath(path_);

    planner_->prepare();
    plan_ = planner_->getPlan();

    rclcpp::Rate r(10);
    for(int i = 0; i < 10; i++) {
      printf("wait for costmap\n");
      rclcpp::spin_some(this->get_node_base_interface());
      r.sleep();
    }

    auto start = std::chrono::system_clock::now();
    int count = 0;
    while (rclcpp::ok()) {
      if (count % 20 == 0) {
        unsigned char *data = costmap_->getCharMap();
        planner_->setCost(data);
      }
      bool result = planner_->iterate();
      auto end = std::chrono::system_clock::now();
      auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      printf("%d iteration = %ldms\n", ++count, ms.count());
      plan_ = planner_->getPlan();
      rclcpp::spin_some(this->get_node_base_interface());
      if (result) {
        break;
      }
    }
    rclcpp::sleep_for(3000ms);
  }
}

}  // namespace cabot_planner
