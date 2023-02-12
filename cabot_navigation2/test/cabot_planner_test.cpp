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

#include <math.h>
#include <memory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav2_navfn_planner/navfn.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/lifecycle_utils.hpp>
#include <nav2_util/node_utils.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
namespace fs = boost::filesystem;
using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{
class Test : public nav2_util::LifecycleNode
{
public:
  explicit Test(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Test() {}

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  void run_test();
  void run_test_local();
  void run_test_bag();

private:
  rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  nav_msgs::msg::Path getPath();

  bool alive_ = false;
  int repeat_times_ = 1;
  std::string bagfile_name_ = "";

  std::string plugin_type_;
  nav2_core::GlobalPlanner::Ptr planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid map_obstacle_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_obstacle_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;
};
}  // namespace cabot_navigation2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_navigation2::Test>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}

namespace cabot_navigation2
{

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

Test::Test(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("cabot_planner_test", "", true, options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Setup the global costmap
  costmap_ros_ =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap", std::string{get_namespace()}, "global_costmap");
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

nav2_util::CallbackReturn Test::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_configure");
  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  map_obstacle_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_obstacle", 10);
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 10);

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  planner_ = std::make_shared<CaBotPlanner>();

  auto node = shared_from_this();
  planner_->configure(node, "CaBot", costmap_ros_->getTfBuffer(), costmap_ros_);

  declare_parameter_if_not_declared(rclcpp_node_, "restart", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(rclcpp_node_, "repeat_times", rclcpp::ParameterValue(repeat_times_));
  rclcpp_node_->get_parameter("repeat_times", repeat_times_);

  declare_parameter_if_not_declared(rclcpp_node_, "bagfile", rclcpp::ParameterValue(""));
  rclcpp_node_->get_parameter("bagfile", bagfile_name_);
  RCLCPP_INFO(get_logger(), "bagfile:=%s", bagfile_name_.c_str());

  callback_handler_ =
    rclcpp_node_->add_on_set_parameters_callback(std::bind(&Test::param_set_callback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult Test::param_set_callback(const std::vector<rclcpp::Parameter> params)
{
  auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

  for (auto && param : params) {
    if (has_parameter(param.get_name())) {
      continue;
    }
    RCLCPP_DEBUG(get_logger(), "change param %s", param.get_name().c_str());

    if (param.get_name() == "restart") {
      if (param.as_bool()) {
        rclcpp::Parameter reset_restart("restart", rclcpp::ParameterValue(true));

        RCLCPP_DEBUG(get_logger(), "thread is %s", alive_ ? "alive" : "not alive");
        if (alive_) {
          alive_ = false;
          RCLCPP_DEBUG(get_logger(), "joining thread");
          thread_->join();
          RCLCPP_DEBUG(get_logger(), "joined");
        }

        RCLCPP_DEBUG(get_logger(), "making new thread");
        thread_ = std::make_unique<std::thread>(
          [&]() {
            RCLCPP_INFO(get_logger(), "run_test");
            run_test();
          });
        set_parameter(reset_restart);
      }
    }

    if (param.get_name() == "repeat_times") {
      repeat_times_ = param.as_int();
    }
  }

  results->successful = true;
  return *results;
}

nav2_util::CallbackReturn Test::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_activate");
  costmap_ros_->on_activate(state);
  planner_->activate();
  plan_publisher_->on_activate();
  path_publisher_->on_activate();
  map_publisher_->on_activate();
  map_obstacle_publisher_->on_activate();

  /*
  timer_ = create_wall_timer(1s, [this]() -> void {
    map_publisher_->publish(map_);
    path_publisher_->publish(path_);
  });
  timer2_ = create_wall_timer(0.033s, [this]() -> void {
    plan_ = std::static_pointer_cast<CaBotPlanner>(planner_)->getPlan();
    plan_publisher_->publish(plan_);
  });
  */

  thread_ = std::make_unique<std::thread>(
    [&]() {
      RCLCPP_INFO(get_logger(), "run_test");
      run_test();
    });

  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn Test::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_deactivate");

  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn Test::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_cleanup");

  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn Test::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_shutdown");

  return nav2_util::CallbackReturn::SUCCESS;
}

void Test::run_test()
{
  alive_ = true;
  RCLCPP_INFO(get_logger(), "bagfile:=%s", bagfile_name_.c_str());
  if (bagfile_name_.empty()) {
    run_test_local();
  } else {
    run_test_bag();
  }
}

void Test::run_test_bag()
{
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = bagfile_name_;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);

  auto topics = reader.get_all_topics_and_types();
  std::map<std::string, std::string> topic_type_map;

  // about metadata
  for (auto t : topics) {
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

  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  nav_msgs::msg::Path navcog_path;
  nav_msgs::msg::OccupancyGrid costmap;
  nav_msgs::msg::OccupancyGrid static_costmap;
  people_msgs::msg::People people;
  people_msgs::msg::People obstacles;
  bool topic_ready[7] = {0, 0, 0, 0, 0, 0, 0};

  while (reader.has_next() && rclcpp::ok()) {
    // serialized data
    auto serialized_message = reader.read_next();
    auto topic = serialized_message->topic_name;

    std::string type = topic_type_map[topic];

    rosbag2_cpp::ConverterTypeSupport type_support;
    type_support.type_support_library = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    type_support.rmw_type_support = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", type_support.type_support_library);

    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = rcutils_get_default_allocator();

    if (topic == "/debug/current_pose") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[0] = true;
    }
    if (topic == "/debug/target_goal") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[1] = true;
    }
    if (topic == "/debug/target_path") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[2] = true;
    }
    if (topic == "/debug/target_people") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[3] = true;
    }
    if (topic == "/debug/target_obstacles") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[4] = true;
    }
    if (topic == "/debug/costmap") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[5] = true;
    }
    if (topic == "/debug/static_costmap") {
      ros_message->message = &start;
      cdr_deserializer_->deserialize(serialized_message, type_support.rmw_type_support, ros_message);
      topic_ready[6] = true;
    }

    bool ready = true;
    for (uint64_t i = 0; i < sizeof(topic_ready); i++) {
      ready = ready && topic_ready[i];
    }
    if (ready) {
      RCLCPP_INFO(get_logger(), "do test");
      // do test
    }
  }
}

void Test::run_test_local()
{
  fs::path base_path = ament_index_cpp::get_package_share_directory("cabot_navigation2");
  base_path /= "test";
  fs::path yaml_path = base_path / "test-cases.yaml";

  YAML::Node doc = YAML::LoadFile(yaml_path.string());
  const YAML::Node & tests = doc["tests"];

  for (uint64_t i = 0; i < tests.size() && alive_; i++) {
    const YAML::Node & test = tests[i];
    auto label = yaml_get_value<std::string>(test, "label");
    auto map = yaml_get_value<std::string>(test, "map");
    std::string map_obstacle = "";
    try {
      map_obstacle = yaml_get_value<std::string>(test, "map_obstacle");
    } catch (std::exception & e) {
    }
    auto path = yaml_get_value<std::vector<float>>(test, "path");
    auto skip = yaml_get_value<bool>(test, "skip");
    if (skip) {continue;}

    nav_msgs::msg::Path path_;
    for (unsigned ing64 j = 0; j < path.size(); j += 2) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = path[j];
      pose.pose.position.y = path[j + 1];
      float yaw = 0;
      if (j < path.size() - 3) {
        yaw = std::atan2(path[j + 3] - path[j + 1], path[j + 2] - path[j]);
      } else if (3 < path.size()) {
        yaw = std::atan2(path[j + 1] - path[j - 1], path[j] - path[j - 2]);
      }
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      path_.poses.push_back(pose);
    }
    path_.header.frame_id = "map";
    RCLCPP_INFO(get_logger(), "publish path");
    path_publisher_->publish(path_);

    fs::path map_path = base_path / map;
    nav2_map_server::LoadParameters yaml;
    if (boost::filesystem::exists(map_path)) {
      yaml = nav2_map_server::loadMapYaml(map_path.string());
      nav2_map_server::loadMapFromFile(yaml, map_);
      RCLCPP_INFO(get_logger(), "publish map");
      map_publisher_->publish(map_);
    } else {
      RCLCPP_INFO(get_logger(), "map file not found\n");
    }


    if (!map_obstacle.empty()) {
      fs::path map_obstacle_path = base_path / map_obstacle;
      nav2_map_server::LoadParameters yaml2;
      if (boost::filesystem::exists(map_obstacle_path)) {
        yaml2 = nav2_map_server::loadMapYaml(map_obstacle_path.string());
        nav2_map_server::loadMapFromFile(yaml2, map_obstacle_);
        RCLCPP_INFO(get_logger(), "publish map obstacle");
        map_obstacle_publisher_->publish(map_obstacle_);
      } else {
        RCLCPP_INFO(get_logger(), "map obstacle file not found\n");
      }
    }

    int rate = 10;
    rclcpp::Rate r(rate);
    for (int i = 0; i < rate && alive_; i++) {
      r.sleep();
    }

    rclcpp::Rate r2(0.3);

    std::chrono::duration<int64, std::ratio<1, 1000000000>> total(0);
    for (int64_t j = 0; j < repeat_times_ && alive_; j++) {
      // for (uint64_t k = 0; k < path_.poses.size() - 1; k++) {
      for (uint64_t k = 0; k < 1; k++) {
        geometry_msgs::msg::PoseStamped start, goal;
        start.pose.position = path_.poses[k].pose.position;
        goal.pose.position = path_.poses.back().pose.position;
        auto t0 = std::chrono::system_clock::now();
        auto path = planner_->createPlan(start, goal);
        auto t1 = std::chrono::system_clock::now();
        total += (t1 - t0);
        plan_publisher_->publish(path);
        r2.sleep();
        /*
        planner_->setParam(
            map_.info.width, map_.info.height, map_.info.origin.position.x,
            map_.info.origin.position.y, map_.info.resolution, mode);

        planner_->setPath(path_);

        planner_->prepare();
        plan_ = planner_->getPlan();

        int count = 0;
        unsigned char *data = costmap_->getCharMap();
        planner_->setCost(data);
        while (rclcpp::ok()) {
          bool result = planner_->iterate();
          rclcpp::spin_some(this->get_node_base_interface());
          count++;
          // r.sleep();
          if (result) {
            break;
          }
        }
        auto end = std::chrono::system_clock::now();
        */
      }
    }
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(total);
    printf("repeat = %d, average duration %ld = ms\n", repeat_times_, ms.count() / repeat_times_);
  }
}
}  // namespace cabot_navigation2
