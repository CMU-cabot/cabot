
#include <boost/filesystem.hpp>
#include "rclcpp/rclcpp.hpp"
#include <nav2_navfn_planner/navfn.hpp>
#include <nav2_map_server/map_io.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "opencv2/flann/flann.hpp"

using namespace std::chrono_literals;
namespace fs = boost::filesystem;

// Planner declaration
namespace cabot_plan {
class Point {
 public:
  Point() {x = 0; y = 0;}
  Point(float _x, float _y) { x = _x; y = _y; }
  mutable float x;
  mutable float y;
  void move(float yaw, float length) {
    x += std::cos(yaw)*length;
    y += std::sin(yaw)*length;
  }
  float yaw(float offset = 0) {
    float yaw = std::atan2(y, x) + offset;
    if (yaw > M_PI) {
      yaw -= M_PI*2;
    }
    if (yaw < -M_PI) {
      yaw += M_PI*2;
    }
    return yaw;
  }
  float hypot() {
    return std::hypot(x, y);
  }
  float distance(Point & other) const {
    return std::hypot(x-other.x, y-other.y);
  }
  Point operator - (const Point & other) const { return Point(x - other.x, y - other.y); }
  void operator -= (const Point & other) { x -= other.x; y -= other.y; }
  Point operator + (const Point & other) const { return Point(x + other.x, y + other.y); }
  void operator += (const Point & other) { x += other.x; y += other.y; }
  Point operator * (float s) const { return Point(x*s, y*s); }
  void operator *= (float s) { x *= s; y *= s; }
};
class Node: public Point {
 public:
  Node(): Point() { anchor.x = x; anchor.y = y;}
  Node(float _x, float _y): Point(_x, _y) { anchor.x = x; anchor.y = y;}
  bool collision = false;
  Point anchor;
};
class Link {
 public:
  Node * n0;
  Node * n1;
  bool collision() {
    return n0->collision || n1->collision;
  }
  float yaw(float offset=0) {
    float yaw = std::atan2(n1->y-n0->y, n1->x-n0->x) + offset;
    if (yaw > M_PI) {
      yaw -= M_PI*2;
    }
    if (yaw < -M_PI) {
      yaw += M_PI*2;
    }
    return yaw;
  }
};
class Obstacle: public Point {
 public:
  Obstacle(): Point() { }
  Obstacle(float _x, float _y, int _index, float _size): Point(_x, _y) { index = _index; size = _size;}

  bool operator < (const Obstacle& rhs ) const {
    return index < rhs.index;
  }

  bool merge(const Obstacle & rhs) const {
    if (invalid) return false;
    if (rhs.invalid) return false;
    auto _dist = std::hypot(x-rhs.x, y-rhs.y);
    auto _size = std::hypot(size, rhs.size);
    if (_dist > 2*_size) return false;

    x = (x*size + rhs.x*rhs.size)/(size+rhs.size);
    y = (y*size + rhs.y*rhs.size)/(size+rhs.size);
    size = _size;
    return true;
  }

  int index;
  mutable float size;
  mutable bool invalid = false;
};
class ObstacleGroup: public Obstacle {
 public:
  ObstacleGroup(): Obstacle() {}
  bool add(Obstacle obstacle) {
    auto result = obstacles_.insert(obstacle);
    return result.second;
  }
  void complete() {
    float tx = 0;
    float ty = 0;
    float ts = 0;
    std::set<Obstacle>::iterator it;
    for(it = obstacles_.begin(); it != obstacles_.end(); it++) {
      tx += it->x;
      ty += it->y;
    }
    tx /= obstacles_.size();
    ty /= obstacles_.size();
    for(it = obstacles_.begin(); it != obstacles_.end(); it++) {
      float s = std::hypot(it->x - tx, it->y - ty);
      if (ts < s) {
        ts = s;
      }
    }
    x = tx;
    y = ty;
    size = ts;
  }
  std::set<Obstacle> obstacles_;
};

class Planner {
 public:
  void setParam(int width, int height, float origin_x, float origin_y, float resolution);
  bool worldToMap(float wx, float wy, float & mx, float & my);
  void mapToWorld(float mx, float my, float & wx, float & wy);
  int getIndex(float x, float y);
  int getIndexByPoint(Point & p);
  void setCost(unsigned char* cost);
  void setPath(nav_msgs::msg::Path path);
  bool plan(std::chrono::duration<int64_t, std::milli> period);
  void prepare();
  bool iterate();
  nav_msgs::msg::Path getPlan(void);
 protected:
  std::vector<Node> findNodesNearObstacle(Obstacle obstacle, float distance);
  std::vector<Link*> findLinksNearObstacle(Obstacle obstacle, float distance);
  std::vector<Node> getNodesFromPath(nav_msgs::msg::Path path);
  void findObstacles();
  void scanObstacleAt(ObstacleGroup & group, float mx, float my, unsigned int cost);
  std::vector<Link> getLinksFromNodes(std::vector<Node> & nodes);
  std::vector<Obstacle> getObstaclesNearNode(Node & node);
 private:
  int width_;
  int height_;
  float origin_x_;
  float origin_y_;
  float resolution_;
  unsigned char* cost_;
  unsigned char* mark_;
  nav_msgs::msg::Path path_;
  std::vector<Node> nodes_;
  std::vector<Link> links_;
  std::set<Obstacle> obstacles_;
  std::set<ObstacleGroup> groups_;
  std::vector<Obstacle> olist_;
  cv::Mat *data_;
  cv::flann::Index *idx_;
};
}

// Planner implementation
namespace cabot_plan {

bool Planner::worldToMap(float wx, float wy, float & mx, float & my)
{
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

void Planner::mapToWorld(float mx, float my, float & wx, float & wy)
{
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
}

int Planner::getIndex(float x, float y) {
  if (x < 0 || y < 0 || width_ < x || height_ < y) {
    return -1;
  }
  return static_cast<int>(x) + static_cast<int>(y) * width_;
}

int Planner::getIndexByPoint(Point & p) {
  return getIndex(p.x, p.y);
}

void Planner::setParam(int width, int height, float origin_x, float origin_y, float resolution) {
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  if (cost_) {
    delete cost_;
  }
  cost_ = new unsigned char[width_*height_];
  if (mark_) {
    delete mark_;  
  }
  mark_ = new unsigned char[width_*height_];
}

void Planner::setCost(unsigned char *cost) {
  unsigned char *p = cost_;
  unsigned char *p2 = mark_;
  for(int x = 0; x < width_; x++) {
    for(int y = 0; y < width_; y++) {
      *p++ = *cost++;
      *p2++ = 0;
    }
  }
  findObstacles();
}

void Planner::findObstacles() {
  // check obstacles only from current position to N meters forward
  // traverse the path
  int COST=252;

  printf("group\n");
  groups_.clear();
  obstacles_.clear();
  std::vector<Node>::iterator it;
  for(it = nodes_.begin(); it != nodes_.end(); it++) {
    int index = getIndexByPoint(*it);
    if (index < 0) {
      continue;
    }
    auto cost = cost_[index];
    // if the path is on an lethal cost (254) cell
    // extract all adjusent lethal cost cells
    if (cost > COST && mark_[index] == 0) {
      ObstacleGroup group;
      scanObstacleAt(group, it->x, it->y, COST);
      group.complete();
      //printf("%.2f %.2f %.2f %ld\n", group.x, group.y, group.size, group.obstacles_.size());
      groups_.insert(group);
    } 
  }

  printf("obstacles\n");
  // find surrounding lethal cost except the obstacles the path is on 
  std::vector<int> marks;
  for(it = nodes_.begin(); it != nodes_.end(); it++) {
    int index = getIndexByPoint(*it);
    if (index < 0) {
      continue;
    }
    auto cost = cost_[index];
    if (cost <= COST) {
      mark_[index] = 1;
      marks.push_back(index);
    }
  }
  int i = 0;
  int MAX = 20;
  int max_size = width_*height_;
  while(i < marks.size()) {
    int index = marks[i++];
    auto cost = cost_[index];
    if (cost > COST) {
      float x = index % width_;
      float y = index / width_;
      obstacles_.insert(Obstacle(x, y, index, 1.6));
      //printf("%.2f %.2f\n", x, y);
    }
    unsigned char current = mark_[index];
    if (current >= MAX) continue;
    if (0 <= index-1 && mark_[index-1] == 0) {
      mark_[index-1]= current + 1;
      marks.push_back(index-1);
    }
    if (index+1 < max_size && mark_[index+1] == 0) {
      mark_[index+1]= current + 1;
      marks.push_back(index+1);
    }
    if (0 <= index-width_ && mark_[index-width_] == 0) {
      mark_[index-width_]= current + 1;
      marks.push_back(index-width_);
    }
    if (index+width_ < max_size && mark_[index+width_] == 0) {
      mark_[index+width_]= current + 1;
      marks.push_back(index+width_);
    }
  }

  printf("obstacles %ld\n", obstacles_.size());
  int n = obstacles_.size();
  if (n == 0) {
    return;
  }
  if (idx_) {
    delete idx_;
  }
  if (data_) {
    delete data_;
  }
  data_ = new cv::Mat(n, 2, CV_32FC1);
  olist_.clear();
  i = 0;
  std::set<Obstacle>::iterator oit;
  for (oit = obstacles_.begin(); oit != obstacles_.end(); ++oit) {
    olist_.push_back(*oit);
    data_->at<float>(i,0) = oit->x;
    data_->at<float>(i,1) = oit->y;
    i++;
  }
  printf("making index %d/%d\n", i, n);
  idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  printf("made index\n");
  //printf("obstacles = %ld\n", obstacles_.size());
}

std::vector<Obstacle> Planner::getObstaclesNearNode(Node & node){
  cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
  query.at<float>(0) = node.x;
  query.at<float>(1) = node.y;
  std::vector<int> indices;
  std::vector<float> dists;
  
  int m = idx_->radiusSearch(query, indices, dists, 100, 100);
  std::vector<Obstacle> list;
  for(int i = 0; i < m; i++) {
    list.push_back(olist_[indices[i]]);
  }
  return list;
}

void Planner::scanObstacleAt(ObstacleGroup & group, float mx, float my, unsigned int cost) {
  int index = getIndex(mx, my);
  if (cost_[index] <= cost) { return; }
  if (mark_[index] != 0) { return; }
  group.add(Obstacle(mx, my, index, 5));
  mark_[index] = 255;
  if (mx+1 < width_) {
    scanObstacleAt(group, mx+1, my, cost);
  }
  if (my+1 < height_) {
    scanObstacleAt(group, mx, my+1, cost);
  }
  if (mx-1 >= 0) {
    scanObstacleAt(group, mx-1, my, cost);
  }
  if (my-1 >= 0) {
    scanObstacleAt(group, mx, my-1, cost);
  }
}

/* test
void Planner::findObstacles() {
  obstacles_.insert(Obstacle(138, 174, 1111, 18));
  obstacles_.insert(Obstacle(50, 112, 1112, 35));
}
*/

/* sample obstacles and clustering
  - did not work well
  - single island of lethal cost should be presented as an obstacle, otherwise
    planned path could be in a local minimum
void Planner::findObstalces() {
  int size = 3;
  for(int j = 0; j < height_; j+=size) {
    for(int i = 0; i < width_; i+=size) {
      auto index = getIndex(i, j);
      float c = cost_[index];
      if (c != 254) {
        continue;
      }
      Obstacle obstacle(i, j, index, size/2.0);
      auto result = obstacles_.insert(obstacle);
    }
  }
  if (obstacles_.size() == 0) return;

  std::set<Obstacle>::iterator it;
  for (it = obstacles_.begin(); it != obstacles_.end(); ++it) {
    if (it->invalid) continue;
    //printf("%.2f %.2f %.2f\n", it->x, it->y, it->size);
  }

  printf("obstacles = %ld\n", obstacles_.size());
  for (it = obstacles_.begin(); it != std::prev(obstacles_.end()); ++it) {
    if (it->invalid) continue;
    std::set<Obstacle>::iterator jt;
    for(jt = std::next(it); jt != obstacles_.end(); ++jt) {
      if (jt->invalid) continue;
      Obstacle oj = *jt;
      if(it->merge(oj)) {
        jt->invalid = true;
      }
    }
  }

  std::set<Obstacle> obstacles;
  for (it = obstacles_.begin(); it != obstacles_.end(); ++it) {
    if (it->invalid) continue;
    obstacles.insert(*it);
    printf("%.2f %.2f %.2f\n", it->x, it->y, it->size);
  }
  obstacles_ = obstacles;
  printf("obstacles = %ld\n", obstacles_.size());
}
*/

void Planner::setPath(nav_msgs::msg::Path path) {
  path_ = path;
  findObstacles();
}

bool Planner::plan(std::chrono::duration<int64_t, std::milli> period) {
  prepare();
  
  auto start = std::chrono::system_clock::now();
  while(true) {
    auto end = std::chrono::system_clock::now();
    if ((end - start) > period) {
      return false;
    }
    return iterate();
    //if (iterate()) {
    //break;
    //}
    //count++;
  }
  return true;
}

void Planner::prepare() {
  nodes_ = getNodesFromPath(path_);
  links_ = getLinksFromNodes(nodes_);
}

bool Planner::iterate() {
  float scale = 0.1;
  float gravity_factor = 1.0;
  float link_spring_factor = 1.0; // spring factor;
  float anchor_spring_factor = 0.01; // spring factor;

  std::vector<Node> newNodes;
  
  Node * n0 = &nodes_[0];
  newNodes.push_back(*n0);
  
  for (unsigned long i = 1; i < nodes_.size();  i++) {
    Node * n1 = &nodes_[i];
    Node newNode;
    newNode.x = n1->x;
    newNode.y = n1->y;

    std::set<ObstacleGroup>::iterator ogit;
    for (ogit = groups_.begin(); ogit != groups_.end(); ++ogit) {
      float d = ogit->distance(*n1);
      d = std::max(0.0f, d - ogit->size);
      float yaw = 0;
      if (d < 0.5) {
        d = 0.5;
        yaw = (*n1-*n0).yaw(+M_PI_2);
      } else {
        yaw = (*n1-*ogit).yaw();
      }
      float m = gravity_factor/d/d*scale;
      newNode.move(yaw, m);
    }

    std::vector<Obstacle> list = getObstaclesNearNode(*n1);
    std::vector<Obstacle>::iterator it;
    for (it = list.begin(); it != list.end(); ++it) {
      if (it->invalid) continue;
      //float d = std::min(0.0, it->distance(*n1) - it->size);
      float d = it->distance(*n1);
      d = std::max(0.0f, d - it->size);
      float yaw = 0;
      if (d < 0.5) {
        d = 0.5;
        yaw = (*n1-*n0).yaw(+M_PI_2);
      } else {
        yaw = (*n1-*it).yaw();
      }
      float m = gravity_factor/d/d*scale;
      newNode.move(yaw, m);
    }

    float d = n1->distance(n1->anchor);
    if (d > 0.1) {
      float yaw = (n1->anchor - *n1).yaw();
      newNode.move(yaw, d * anchor_spring_factor * scale);
    }

    if (i < nodes_.size()-1) {
      Node * n2 = &nodes_[i+1];
      
      d = n0->distance(*n1);
      if (d > 0.1) {
        float yaw = (*n0 - *n1).yaw();
        newNode.move(yaw, d * link_spring_factor * scale);
      }
      //printf("%d: %.2f %.2f ", i, d, yaw);
      d = n2->distance(*n1);
      if (d > 0.1) {
        float yaw = (*n2 - *n1).yaw();
        newNode.move(yaw, d * link_spring_factor * scale);
      }
      //printf("%d: %.2f %.2f\n", i, d, yaw);
      //printf("(%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)\n", n0->x, n0->y, n1->x, n1->y, newNode.x, newNode.y, n2->x, n2->y);
    }

    n0 = n1;

    newNodes.push_back(newNode);
  }

  bool flag = true;
  for (unsigned long i = 0; i < nodes_.size();  i++) {
    if (nodes_[i].x - newNodes[i].x > 0.02 || 
        nodes_[i].y - newNodes[i].y > 0.02) {
      flag = false;          
    }
    nodes_[i].x = newNodes[i].x;
    nodes_[i].y = newNodes[i].y;
  }
  
  //printf("----------------------------------------------\n");
  return flag;
}

std::vector<Node> Planner::findNodesNearObstacle(Obstacle obstacle, float distance) {
  std::vector<Node> nodes;
  for(unsigned long i = 0; i < nodes_.size(); i++) {
    Node & node = nodes_[i];
    float dist = obstacle.distance(node);
    if (dist < distance) {
      nodes.push_back(node);
    }
  }
  return nodes;
}

std::vector<Link*> Planner::findLinksNearObstacle(Obstacle obstacle, float distance) {
  std::vector<Link*> links;
  for(unsigned long i = 0; i < links_.size(); i++) {
    Link & link = links_[i];
    float dist = obstacle.distance(*link.n0);
    if (dist < distance) {
      links.push_back(&link);
    }
  }
  return links;
}

/*
 * generate map coordinate points
 */
std::vector<Node> Planner::getNodesFromPath(nav_msgs::msg::Path path) {
  std::vector<Node> nodes;
  
  // push initial pose
  auto p0 = path.poses[0].pose.position;
  for(long unsigned int i = 1; i < path.poses.size(); i++) {
    auto p1 = path.poses[i].pose.position;

    auto dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
    int N = std::ceil(dist / 0.10);

    for(int j = 0; j <= N; j++) {
      // deal with the last pose
      if (j == N && i < path.poses.size() - 1) {
        continue;
      }
      float mx=0, my=0;
      worldToMap((p0.x * (N-j) + p1.x * j) / N, (p0.y * (N-j) + p1.y * j) / N, mx, my);
      nodes.push_back(Node(mx, my));
    }
    p0 = p1;
  }

  return nodes;
}

std::vector<Link> Planner::getLinksFromNodes(std::vector<Node> & nodes) {
  std::vector<Link> links;
  Node *n0 = &nodes[0];
  for(long unsigned int i = 1; i < nodes.size(); i++) {
    Node *n1 = &nodes[i];
    Link link;
    link.n0 = n0;
    link.n1 = n1;
    links.push_back(link);
    n0 = n1;
  }
  return links;
}

nav_msgs::msg::Path Planner::getPlan(void) {
  nav_msgs::msg::Path ret;
  ret.header.frame_id = "map";

  auto mp0 = nodes_[0];
  for(long unsigned int i = 1; i < nodes_.size(); i++) {
    auto mp1 = nodes_[i];
    geometry_msgs::msg::PoseStamped wp;
    float mx, my;
    mapToWorld(mp0.x, mp0.y, mx, my);
    wp.pose.position.x = mx;
    wp.pose.position.y = my;
    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(mp1.y-mp0.y, mp1.x-mp0.x));
    wp.pose.orientation.x = q.x();
    wp.pose.orientation.y = q.y();
    wp.pose.orientation.z = q.z();
    wp.pose.orientation.w = q.w();
    ret.poses.push_back(wp);

    mp0 = mp1;
  }
  return ret;
}
}

namespace cabot_plan {
class Test: public rclcpp::Node {
 public:
  bool
  worldToMap(float wx, float wy, unsigned int & mx, unsigned int & my)
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
  mapToWorld(float mx, float my, float & wx, float & wy)
  {
    wx = map_.info.origin.position.x + mx * map_.info.resolution;
    wy = map_.info.origin.position.y + my * map_.info.resolution;
  }
  
  Test(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
      rclcpp::Node("cabot_plan", "", options){
    fs::path yaml_path = ament_index_cpp::get_package_share_directory("cabot_navigation2");
    yaml_path /= "test/test-map.yaml";

    // Setup the global costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap", std::string{get_namespace()}, "global_costmap");
    // Launch a thread to run the costmap node
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    rclcpp_lifecycle::State state;
    costmap_ros_->on_configure(state);
    costmap_ = costmap_ros_->getCostmap();
    costmap_ros_->on_activate(state);

    nav2_map_server::LoadParameters yaml;
    if (boost::filesystem::exists(yaml_path)) {
      yaml = nav2_map_server::loadMapYaml(yaml_path.string());
      nav2_map_server::loadMapFromFile(yaml, map_);
    } else {
      printf("file not found\n");
    }

    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
    timer_ = create_wall_timer(
        1s,
        [this]() -> void {
          map_publisher_->publish(map_);
          path_publisher_->publish(path_);
        });
    timer2_ = create_wall_timer(
        0.033s,
        [this]() -> void {
          plan_publisher_->publish(plan_);
        });

    printf("wait for costmap\n");
    rclcpp::Rate r(100);
    while (!costmap_ros_->isCurrent()) {
      rclcpp::spin_some(this->get_node_base_interface());
      r.sleep();
    }

    printf("making a plan\n");

    float wsx = -4.0;
    float wsy = -4.0;
    float wgx = 4.0;
    float wgy = 3.0;

    unsigned int sx, sy;
    worldToMap(wsx, wsy, sx, sy);
    unsigned int gx, gy;
    worldToMap(wgx, wgy, gx, gy);

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

    planner_ = std::make_unique<Planner>();
    planner_->setParam(map_.info.width, map_.info.height, map_.info.origin.position.x, map_.info.origin.position.y, map_.info.resolution);

    planner_->setPath(path_);

    planner_->prepare();
    plan_ = planner_->getPlan();
    rclcpp::sleep_for(3000ms);
    auto start = std::chrono::system_clock::now();
    int count = 0;
    while(rclcpp::ok()) {
      //if (!planner_->plan(100ms)) {
      //printf("Plan timeout\n");
      //}
      if (count%20 == 0) {
        unsigned char *data = costmap_->getCharMap();
        planner_->setCost(data);
      }
      bool result = planner_->iterate();
      auto end = std::chrono::system_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
      printf("%d iteration = %ldms\n", ++count, ms.count());
      plan_ = planner_->getPlan();
      rclcpp::spin_some(this->get_node_base_interface());
      if (result) {
        break;
      }
    }
  }

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
  nav2_costmap_2d::Costmap2D * costmap_;  
};
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_plan::Test>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}
  
