
#include <boost/filesystem.hpp>
#include "rclcpp/rclcpp.hpp"
#include <nav2_navfn_planner/navfn.hpp>
#include <nav2_map_server/map_io.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <nav_msgs/msg/path.hpp>


using namespace std::chrono_literals;
namespace fs = boost::filesystem;

namespace cabot_plan {
class Test: public rclcpp::Node {
 public:
  bool
  worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
  {
    if (wx < map_.info.origin.position.x || wy < map_.info.origin.position.y) {
      return false;
    }
    
    mx = static_cast<int>(
        std::round((wx - map_.info.origin.position.x) / map_.info.resolution));
    my = static_cast<int>(
        std::round((wy - map_.info.origin.position.y) / map_.info.resolution));
    
    if (mx < map_.info.width && my < map_.info.height) {
      return true;
    }
    return false;
  }

  void
  mapToWorld(double mx, double my, double & wx, double & wy)
  {
    wx = map_.info.origin.position.x + mx * map_.info.resolution;
    wy = map_.info.origin.position.y + my * map_.info.resolution;
  }
  
  bool
  getPlanFromPotential(
      const geometry_msgs::msg::Pose & goal,
      const geometry_msgs::msg::Pose & start,
      nav_msgs::msg::Path & plan)
  {
    printf("getPlanFromPotential\n");
           
    // clear the plan, just in case
    plan.poses.clear();
    
    // Goal should be in global frame
    double wx = goal.position.x;
    double wy = goal.position.y;
    
    // the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if (!worldToMap(wx, wy, mx, my)) {
      printf("cannot convert\n");
      return false;
    }
    
    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    wx = start.position.x;
    wy = start.position.y;
    if (!worldToMap(wx, wy, mx, my)) {
      printf("cannot convert\n");
      return false;
    }
    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;
    
    planner_->setStart(map_goal);
    
    const int & max_cycles = (map_.info.width >= map_.info.height) ?
                             (map_.info.width * 4) : (map_.info.height * 4);

    int path_len = planner_->calcPath(max_cycles, map_start);

    printf("path length = %d\n", path_len);
    if (path_len == 0) {
      return false;
    }
    
    auto cost = planner_->getLastPathCost();
    
    // extract the plan
    float * x = planner_->getPathX();
    float * y = planner_->getPathY();
    int len = planner_->getPathLen();
    
    for (int i = len - 1; i >= 0; --i) {
      // convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);
      
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.poses.push_back(pose);
    }
    
    return !plan.poses.empty();
  }


  void getPotential() {
    float max = 0;
    float min = 10000000;
    for (int i = 0; i < pot_.data.size(); i++) {
      float pot = planner_->potarr[i];
      if (pot == 10000000000.0) {
        continue;
      }
      if (max < pot) {
        max = pot;
      }
      if (min > pot) {
        min = pot;
      }
    }
    for (int i = 0; i < pot_.data.size(); i++) {
      pot_.data[i] = (planner_->potarr[i] -min)/max*255;
      if (planner_->potarr[i] > max) {
        pot_.data[i] = 255;
      } 
    }
    printf("max=%.2f  min=%.2f\n",max, min);
  }

  
  Test(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
      rclcpp::Node("cabot_plan", "", options){
    fs::path yaml_path = ament_index_cpp::get_package_share_directory("cabot_navigation2");
    yaml_path /= "test/test-map.yaml";

    nav2_map_server::LoadParameters yaml;
    if (boost::filesystem::exists(yaml_path)) {
      yaml = nav2_map_server::loadMapYaml(yaml_path.string());
      nav2_map_server::loadMapFromFile(yaml, map_);
      pot_ = map_;
    } else {
      printf("file not found\n");
    }

    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    pot_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("pot", 1);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", 1);
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);
    timer_ = create_wall_timer(
        1s,
        [this]() -> void {
          map_publisher_->publish(map_);
          pot_publisher_->publish(pot_);
          path_publisher_->publish(path_);
          plan_publisher_->publish(plan_);
        });

    planner_ = std::make_unique<nav2_navfn_planner::NavFn>(map_.info.width, map_.info.height);

    planner_->setNavArr(map_.info.width, map_.info.height);

    unsigned char *data = new unsigned char[map_.info.width * map_.info.height];
    for (int i = 0; i < map_.data.size(); i++) {
      data[i] = map_.data[i];
      if (data[i] < yaml.free_thresh * 255) {
        data[i] = 0;
      } else {
        data [i] = 254;
      }
    }
    planner_->setCostmap(data, true, true);

    float wsx = -4.0;
    float wsy = -4.0;
    float wgx = 4.0;
    float wgy = 4.0;
    int map_start[2];
    unsigned int sx, sy;
    worldToMap(wsx, wsy, sx, sy);
    map_start[0] = sx;
    map_start[1] = sy;
    int map_goal[2];
    unsigned int gx, gy;
    worldToMap(wgx, wgy, gx, gy);
    map_goal[0] = gx;
    map_goal[1] = gy;

    printf("start = %d, %d | goal = %d, %d\n", sx, sy, gx, gy);
    
    geometry_msgs::msg::PoseStamped pose0;
    pose0.pose.position.x = wsx;
    pose0.pose.position.y = wsy;
    path_.poses.push_back(pose0);
    geometry_msgs::msg::PoseStamped pose1;
    pose1.pose.position.x = 4.0;
    pose1.pose.position.y = -4.0;
    path_.poses.push_back(pose1);
    geometry_msgs::msg::PoseStamped pose2;
    pose2.pose.position.x = 4.0;
    pose2.pose.position.y = -1.0;
    path_.poses.push_back(pose2);
    geometry_msgs::msg::PoseStamped pose3;
    pose3.pose.position.x = 0;
    pose3.pose.position.y = -1;
    path_.poses.push_back(pose3);
    geometry_msgs::msg::PoseStamped pose4;
    pose4.pose.position.x = -0.5;
    pose4.pose.position.y = 3;
    path_.poses.push_back(pose4);
    geometry_msgs::msg::PoseStamped pose5;
    pose5.pose.position.x = wgx;
    pose5.pose.position.y = wgy;
    path_.poses.push_back(pose5);
    path_.header.frame_id = "map";

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    planner_->setupNavFn(true);

    int n = 0;
    auto p0 = path_.poses[path_.poses.size()-1].pose.position;
    for(int i = 4; i >= 0; i--) {
      auto p1 = path_.poses[i].pose.position;
      static int MAX = 1000;
      int prev = -1;
      for(int j = 0; j < MAX; j++) {
        if (i == 0 && j > MAX*.95) {
          break;
        }
        float wx = (p0.x * (MAX-j) + p1.x * (j)) / MAX;
        float wy = (p0.y * (MAX-j) + p1.y * (j)) / MAX;
        unsigned int mx, my;
        worldToMap(wx, wy, mx, my);
        int index = mx + my * map_.info.width;
        //planner_->potarr[index] = n;
        //n+=10;
        if (prev != index && data[index] == 0) {
          planner_->curP[planner_->curPe++] = index;
          planner_->pending[index] = true;
          prev = index;
        }
      }
      p0 = p1;
    }
    
    planner_->propNavFnAstar(std::max(planner_->nx * planner_->ny / 20, planner_->nx + planner_->ny));
    //planner_->propNavFnDijkstra(std::max(planner_->nx * planner_->ny / 20, planner_->nx + planner_->ny), true);
    getPotential();
    
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = wsx;
    start_pose.position.y = wsy;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = wgx;
    goal_pose.position.y = wgy;

    getPlanFromPotential(goal_pose, start_pose, plan_);
    plan_.header.frame_id = "map";

    /*
      auto planner_ = std::make_unique<NavFn>(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
    */
  }

  std::unique_ptr<nav2_navfn_planner::NavFn> planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid pot_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path plan_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pot_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
};
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_plan::Test>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}
  
