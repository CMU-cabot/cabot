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

#include "cabot_navigation2/cabot_planner_param.hpp"

#include <mutex>

namespace cabot_navigation2 {

CaBotPlan::CaBotPlan(CaBotPlannerParam & param_): param(param_) {
    nodes_backup = param.getNodes();
    resetNodes();
    findIndex();
}

void CaBotPlan::resetNodes() {
  nodes.clear();
  for(unsigned long i = 0; i < nodes_backup.size(); i++) {
    nodes_backup[i].reset();
    nodes.push_back(nodes_backup[i]);
  }
}

nav_msgs::msg::Path CaBotPlan::getPlan(bool normalized, float normalize_length) {
  nav_msgs::msg::Path ret;
  ret.header.frame_id = "map";

  if (nodes.size() == 0) {
    return ret;
  }

  if (normalized == false) {
    auto mp0 = nodes[0];
    for (long unsigned int i = 1; i < nodes.size(); i++) {
      auto mp1 = nodes[i];
      float dy = mp1.y - mp0.y;
      float dx = mp1.x - mp0.x;
      if (dx == 0 && dy == 0) { continue; }
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);

      float mx, my;
      param.mapToWorld(mp0.x, mp0.y, mx, my);

      geometry_msgs::msg::PoseStamped wp;
      wp.pose.position.x = mx;
      wp.pose.position.y = my;
      wp.pose.orientation.x = q.x();
      wp.pose.orientation.y = q.y();
      wp.pose.orientation.z = q.z();
      wp.pose.orientation.w = q.w();
      ret.poses.push_back(wp);

      mp0 = mp1;
    }
  }
  else {
    auto mp0 = nodes[0];
    for (long unsigned int i = 1; i < nodes.size(); i++) {
      auto mp1 = nodes[i];
      float dy = mp1.y - mp0.y;
      float dx = mp1.x - mp0.x;
      if (dx == 0 && dy == 0) { continue; }
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      
      float mx, my;
      param.mapToWorld(mp0.x, mp0.y, mx, my);

      geometry_msgs::msg::PoseStamped wp;
      wp.pose.position.x = mx;
      wp.pose.position.y = my;
      wp.pose.orientation.x = q.x();
      wp.pose.orientation.y = q.y();
      wp.pose.orientation.z = q.z();
      wp.pose.orientation.w = q.w();
      ret.poses.push_back(wp);

      float dist = std::hypot(dx, dy);
      if (dist > normalize_length / param.resolution) {
        mp0 = Node(mp0.x + std::cos(yaw)*normalize_length / param.resolution, mp0.y + std::sin(yaw)*normalize_length / param.resolution);
        i--;
        continue;
      }

      mp0 = mp1;
    }
  }
  return ret;
}

void CaBotPlan::findIndex() {
  // find start/end index around the start
  float mx, my;
  param.worldToMap(param.start.pose.position.x, param.start.pose.position.y, mx, my);
  Node start_node(mx, my);
  start_index = 0;
  end_index = ULLONG_MAX;
  float min_dist = 1000;
  float optimize_distance_from_start = param.options.optimize_distance_from_start / param.resolution;
  float distance_from_start = 0;
  for (unsigned long i = 0; i < nodes.size()-1; i++) {
    Node *n0 = &nodes[i];
    if (start_node.distance(*n0)< min_dist) {
      start_index = i;
      min_dist = start_node.distance(*n0);
    }
  }
  for (unsigned long i = start_index; i < nodes.size()-1; i++) {
    Node *n0 = &nodes[i];
    Node *n1 = &nodes[i+1];
    distance_from_start += n0->distance(*n1);
    end_index = i+2;
    if (optimize_distance_from_start < distance_from_start) {
      break;
    }
  }  
  /*
  if (start_index < (unsigned long)(5.0/options_.initial_node_interval_meter)) {
    start_index = 0;
  } else {
    start_index = start_index - (int)(5.0/options_.initial_node_interval_meter);
  }
  */
  RCLCPP_INFO(param.logger, "start_index=%ld, end_index=%ld, size=%ld", start_index, end_index, nodes.size());
}

void CaBotPlan::adjust() {
  bool changed = false;
  int devide_link_cell_interval_threshold = param.options.devide_link_cell_interval_threshold;
  int shrink_link_cell_interval_threshold = param.options.initial_node_interval_meter / 2.0 / param.resolution;
  for (unsigned long i = 0; i < nodes.size() - 1; i++) {
    if (nodes_backup.size() * 2 < nodes.size()) {
      break;
    }
    Node *n0 = &nodes[i];
    Node *n1 = &nodes[i + 1];

    auto distance = n0->distance(*n1);
    if (distance > devide_link_cell_interval_threshold) {
      Node newNode;
      newNode.x = (n0->x + n1->x) / 2.0;
      newNode.y = (n0->y + n1->y) / 2.0;
      newNode.anchor.x = (n0->anchor.x + n1->anchor.y) / 2.0;
      newNode.anchor.y = (n0->anchor.y + n1->anchor.y) / 2.0;
      newNode.angle = n0->angle;
      //RCLCPP_INFO(param.logger, "newNode2 [%ld] x = %.2f, y = %.2f", i + 1, newNode.x, newNode.y);
      if (i < start_index) {
        start_index++;
        changed = true;
      }
      if (i < end_index) {
        end_index++;
        changed = true;
      }
      i++;
      nodes.insert(nodes.begin() + i, newNode);
    }
    if (distance < shrink_link_cell_interval_threshold) {
      nodes.erase(nodes.begin() + i);
      if (i < start_index) {
        start_index--;
        changed = true;
      }
      if (i < end_index) {
        end_index--;
        changed = true;
      }
      i--;
    }
  }
  if (changed) {
    RCLCPP_INFO(param.logger, "start_index=%ld, end_index=%ld", start_index, end_index);
  }
}

float CaBotPlan::length() {
  float total = 0;
  for(unsigned long i = 0; i < nodes.size()-1; i++) {
    Node *n0 = &nodes[i];
    Node *n1 = &nodes[i + 1];
    total += n0->distance(*n1);
  }
  return total;
}

std::vector<Node> CaBotPlan::getTargetNodes() {
  std::vector<Node> temp;
  for(unsigned long i = start_index; i < end_index; i++) {
    temp.push_back(nodes[i]);
  }
  return temp;
}

///////////////////////////

CaBotPlannerParam::CaBotPlannerParam(CaBotPlannerOptions &options_, 
  const geometry_msgs::msg::PoseStamped & start_,
  const geometry_msgs::msg::PoseStamped & goal_,
  nav_msgs::msg::Path navcog_path_,
  people_msgs::msg::People::SharedPtr people_msg_ptr_,
  people_msgs::msg::People::SharedPtr obstacles_msg_ptr_,
  nav2_costmap_2d::Costmap2D *costmap_,
  nav2_costmap_2d::Costmap2D *static_costmap_
  ):
options(options_),
start(start_),
goal(goal_),
navcog_path(navcog_path_),
costmap(costmap_),
static_costmap(static_costmap_),
cost(nullptr),
static_cost(nullptr),
mark(nullptr),
data_(nullptr),
data_non_collision_(nullptr),
idx_(nullptr),
idx_non_collision_(nullptr)
{
    if (people_msg_ptr_ != nullptr) people_msg = *people_msg_ptr_;
    if (obstacles_msg_ptr_ != nullptr) obstacles_msg = *obstacles_msg_ptr_;

    assert(costmap->getSizeInCellsX() == static_costmap->getSizeInCellsX());
    assert(costmap->getSizeInCellsY() == static_costmap->getSizeInCellsY());

    width = costmap->getSizeInCellsX();
    height = costmap->getSizeInCellsY();
    resolution = costmap->getResolution();
    origin_x = costmap->getOriginX();
    origin_y = costmap->getOriginY();

    allocate();
    // get cost from costmap
    //  1. static map only
    //  2. layered cost (all layers)
    // then remove cost around moving people/obstacle
    setCost();
}

CaBotPlannerParam::~CaBotPlannerParam() {
  deallocate();
}


void CaBotPlannerParam::allocate() {
  auto size = width * height;
  cost = new unsigned char[size];
  static_cost = new unsigned char[size];
  mark = new unsigned short[size];
}

void CaBotPlannerParam::deallocate() {
  if (cost != nullptr) delete cost;
  if (static_cost != nullptr) delete static_cost;
  if (mark != nullptr) delete mark;
  if (data_) delete data_;
  if (data_non_collision_) delete data_non_collision_;
  if (idx_) delete idx_;
  if (idx_non_collision_) delete idx_non_collision_;
}


void CaBotPlannerParam::setCost() {
  unsigned char *cost_ = costmap->getCharMap();
  unsigned char *static_cost_ = static_costmap->getCharMap();
  unsigned char *p1 = cost;
  unsigned char *p2 = static_cost;
  unsigned short *p3 = mark;

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      *p1++ = *cost_++;
      *p2++ = *static_cost_++;
      *p3++ = 0;
    }
  }

  // ignore moving people/obstacles
  for (auto it = people_msg.people.begin(); it != people_msg.people.end(); it++) {
    bool stationary = (std::find(it->tags.begin(), it->tags.end(), "stationary") != it->tags.end());
    if (!stationary) {
      clearCostAround(*it);
    }
  }
  for (auto it = obstacles_msg.people.begin(); it != obstacles_msg.people.end(); it++) {
    bool stationary = (std::find(it->tags.begin(), it->tags.end(), "stationary") != it->tags.end());
    if (!stationary) {
      clearCostAround(*it);
    }
  }

  int count = 0;
  int scount = 0;
  for(int i = 0; i < width*height; i++) {
    if (cost[i] > 0) count++;
    if (static_cost[i] > 0) scount++;
  }
  RCLCPP_INFO(logger, "cost non zero count = %d, static cost non zero count = %d", count, scount);

}

bool CaBotPlannerParam::adjustPath() {
  path = normalizedPath(navcog_path);
  if (path.poses.empty()) return false;

  PathEstimateOptions pe_options;
  pe_options.path_adjusted_center = 0.5;

  estimatePathWidthAndAdjust(path, costmap, pe_options);
  if (options.adjust_start) {
    path = adjustedPathByStart(path, start);
  }

  path.poses.push_back(goal);
  return true;
}

void CaBotPlannerParam::clearCostAround(people_msgs::msg::Person &person) {
  int cost_lethal_threshold = options.cost_lethal_threshold;
  int max_obstacle_scan_distance = options.max_obstacle_scan_distance;

  float mx, my;
  if (!worldToMap(person.position.x, person.position.y, mx, my)) {
    return;
  }
  RCLCPP_INFO(logger, "clearCostAround %.2f %.2f \n %s", mx, my, rosidl_generator_traits::to_yaml(person).c_str());

  ObstacleGroup group;
  scanObstacleAt(group, mx, my, cost_lethal_threshold, max_obstacle_scan_distance);

  for (auto obstacle = group.obstacles_.begin(); obstacle != group.obstacles_.end(); obstacle++) {
    cost[obstacle->index] = 0;
  }
}


void CaBotPlannerParam::scanObstacleAt(ObstacleGroup &group, float mx, float my, unsigned int min_obstacle_cost, float max_dist) {
  std::queue<std::pair<Obstacle, float>> queue;
  int index = getIndex(mx, my);
  if (index < 0) return;
  queue.push(std::pair<Obstacle, int>(Obstacle(mx, my, index), 0));

  while(queue.size() > 0) {
    auto entry = queue.front();
    auto temp = entry.first;
    queue.pop();
    //RCLCPP_INFO(logger, "queue.size=%ld, cost=%d, mark=%d, (%.2f %.2f) [%d], %d", 
    //             queue.size(), cost_[temp.index], mark_[temp.index], temp.x, temp.y, temp.index, entry.second);
    if (entry.second > max_dist) {
      continue;
    }
    if (mark[temp.index] == 65535) {
      continue;
    }
    mark[temp.index] = 65535;
    if (cost[temp.index] < min_obstacle_cost || static_cost[temp.index] >= min_obstacle_cost) {
      continue;
    }
    auto obstacle = Obstacle(temp.x, temp.y, temp.index, false, cost[temp.index], temp.size, true);
    group.add(obstacle);

    index = getIndex(temp.x+1, temp.y);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x+1, temp.y, index), entry.second+1));
    }
    index = getIndex(temp.x+1, temp.y+1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x+1, temp.y+1, index), entry.second+sqrt(2)));
    }
    index = getIndex(temp.x, temp.y+1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x, temp.y+1, index), entry.second+1));
    }
    index = getIndex(temp.x-1, temp.y+1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x-1, temp.y+1, index), entry.second+sqrt(2)));
    }
    index = getIndex(temp.x-1, temp.y);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x-1, temp.y, index), entry.second+1));
    }
    index = getIndex(temp.x-1, temp.y-1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x-1, temp.y-1, index), entry.second+sqrt(2)));
    }
    index = getIndex(temp.x, temp.y-1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x, temp.y-1, index), entry.second+1));
    }
    index = getIndex(temp.x+1, temp.y-1);
    if (index >= 0) {
      queue.push(std::pair<Obstacle, int>(Obstacle(temp.x+1, temp.y-1, index), entry.second+sqrt(2)));
    }
  }
}

bool CaBotPlannerParam::worldToMap(float wx, float wy, float &mx, float &my) const {
  auto origin_x = costmap->getOriginX();
  auto origin_y = costmap->getOriginY();
  auto resolution = costmap->getResolution();
  auto width = costmap->getSizeInCellsX();
  auto height = costmap->getSizeInCellsY();
  mx = (wx - origin_x) / resolution;
  my = (wy - origin_y) / resolution;

  if (0 <= mx && mx < width && 0 <= my && my < height) {
    return true;
  }
  return false;
}

void CaBotPlannerParam::mapToWorld(float mx, float my, float &wx, float &wy) const {
  auto origin_x = costmap->getOriginX();
  auto origin_y = costmap->getOriginY();
  auto resolution = costmap->getResolution();
  wx = origin_x + mx * resolution;
  wy = origin_y + my * resolution;
}
int CaBotPlannerParam::getIndex(float x, float y) const {
  int width = costmap->getSizeInCellsX();
  int height = costmap->getSizeInCellsY();
  int ix = static_cast<int>(x);
  int iy = static_cast<int>(y);
  if (ix < 0 || iy < 0 || width <= ix || height <= iy) {
    return -(ix + iy * width);
  }
  return ix + iy * width;
}

int CaBotPlannerParam::getIndexByPoint(Point &p) const { 
  return getIndex(p.x, p.y); 
}

std::vector<Node> CaBotPlannerParam::getNodes() const {
  std::vector<Node> nodes;
  auto initial_node_interval_meter = options.initial_node_interval_meter;

  // push initial pose
  auto p0 = path.poses[0].pose.position;
  auto p1 = path.poses[0].pose.position;
  float mx = 0, my = 0;
  for (long unsigned int i = 1; i < path.poses.size(); i++) {
    p1 = path.poses[i].pose.position;

    auto dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
    int N = std::round(dist / initial_node_interval_meter);
    if (N == 0) continue;

    for (int j = 0; j <= N; j++) {
      // deal with the last pose
      if (j == N && i < path.poses.size() - 1) {
        continue;
      }
      worldToMap((p0.x * (N - j) + p1.x * j) / N, (p0.y * (N - j) + p1.y * j) / N, mx, my);
      Node temp = Node(mx, my);
      temp.fixed = (j == 0) || (j == N);
      nodes.push_back(temp);
    }
    p0 = p1;
  }
  worldToMap(p1.x, p1.y, mx, my);
  Node temp = Node(mx, my);
  temp.fixed = true;
  nodes.push_back(temp);

  // check if the goal (last node) is on lethal area and remove it until it can be reached
  Node* prev = nullptr;
  double dist = 0;
  do {
    auto index = getIndexByPoint(nodes.back());
    if (index > 0 && static_cost[index] >= options.cost_pass_threshold) {
      if (prev) {
        dist += prev->distance(nodes.back()) * resolution;
      }
      prev = &nodes.back();
      nodes.pop_back();
      RCLCPP_WARN(logger, "remove last node, dist=%.2f", dist);
    } else {
      break;
    }
  } while (true);

  return nodes;
}


bool isEdge(unsigned char* cost, int width, int height, int index, int target_cost) {
  int max_size = width*height;
  if (index < 0 || max_size <= index) return false;
  unsigned char cost_center = cost[index];
  unsigned char cost_left = target_cost;
  unsigned char cost_right = target_cost;
  unsigned char cost_up = target_cost;
  unsigned char cost_down = target_cost;
  if (0 <= index - 1)           cost_left = cost[index - 1];
  if (index + 1 < max_size)     cost_right = cost[index + 1];
  if (0 <= index - width)       cost_up = cost[index - width];
  if (index + width < max_size) cost_down = cost[index + width];
  return cost_center == target_cost && (cost_left < target_cost || cost_up < target_cost || cost_right < target_cost || cost_down < target_cost);
}

/*
findObstacles scans the costmap to determine obstacles
  Obstacle class: point where the cost is higher than threthold
  ObstacleGroup class: points that all in a single adjucented region

1. find static obstacles near the path
2. find dynamic obstacles near the path
3. create flann index for obstacles (not group obstacles) and point cloud for debug
*/
void CaBotPlannerParam::findObstacles(std::vector<Node> nodes) {
  // check obstacles only from current position to N meters forward
  int cost_lethal_threshold = options.cost_lethal_threshold;
  int max_obstacle_scan_distance = options.max_obstacle_scan_distance;
  RCLCPP_INFO(logger, "nodes.size = %ld", nodes.size());

  groups.clear();
  obstacles.clear();

  // 1. find static obstacles near the path
  {
    memset(mark, 0, width * height * sizeof(unsigned short));
    std::vector<int> marks;
    for (unsigned long i = 0; i < nodes.size(); i++) {
      int index = getIndexByPoint(nodes[i]);
      if (index < 0) {
        continue;
      }
      mark[index] = 1;
      marks.push_back(index);
    }
    unsigned long i = 0;
    long max_size = width * height;
    std::set<Obstacle> staticObstacleSet;
    while (i < marks.size()) {
      long index = marks[i++];
      float x = index % width;
      float y = index / width;

      unsigned char static_cost_value = static_cost[index];
      unsigned short current = mark[index];

      if (current > max_obstacle_scan_distance) continue;

      if (static_cost_value >= cost_lethal_threshold) {
        if (isEdge(static_cost, width, height, index, 253)) {
          staticObstacleSet.insert(Obstacle(x, y, index, true, static_cost_value));
        }
        if (isEdge(static_cost, width, height, index, 254)) {
          staticObstacleSet.insert(Obstacle(x, y, index, true, static_cost_value));
        }
      }

      if (0 <= index - 1 && mark[index - 1] == 0) {
        mark[index - 1] = current + 1;
        marks.push_back(index - 1);
      }
      if (index + 1 < max_size && mark[index + 1] == 0) {
        mark[index + 1] = current + 1;
        marks.push_back(index + 1);
      }
      if (0 <= index - width && mark[index - width] == 0) {
        mark[index - width] = current + 1;
        marks.push_back(index - width);
      }
      if (index + width < max_size && mark[index + width] == 0) {
        mark[index + width] = current + 1;
        marks.push_back(index + width);
      }
    }

    RCLCPP_INFO(logger, "staticObstacleSet size = %ld", staticObstacleSet.size());
    obstacles.insert(obstacles.end(), staticObstacleSet.begin(), staticObstacleSet.end());
  }

  // 2. find dynamic obstacles near the path
  {

    memset(mark, 0, width*height*sizeof(unsigned short));
    std::vector<int> marks;

    for (unsigned long i = 0; i < nodes.size(); i++) {
      int index = getIndexByPoint(nodes[i]);
      if (index < 0) {
        continue;
      }
      auto cost_value = cost[index];
      mark[index] = 1;
      marks.push_back(index);
      if (cost_value >= cost_lethal_threshold) {
        ObstacleGroup group;
        scanObstacleAt(group, nodes[i].x, nodes[i].y, cost_lethal_threshold, max_obstacle_scan_distance);
        if (group.complete()) {
          group.index = getIndexByPoint(group);
          group.collision = true;
          RCLCPP_INFO(logger, "Group Obstacle %.2f %.2f %.2f %ld", group.x, group.y, group.size, group.obstacles_.size());
          groups.push_back(group);
        }
      }
    }

    unsigned long i = 0;
    long max_size = width * height;
    std::set<Obstacle> obstacleSet;
    while (i < marks.size()) {
      long index = marks[i++];
      float x = index % width;
      float y = index / width;

      unsigned char cost_value = cost[index];
      unsigned char static_cost_value = static_cost[index];
      unsigned short current = mark[index];

      if (current > max_obstacle_scan_distance) continue;

      if (static_cost_value < cost_lethal_threshold) {
        if (cost_value >= cost_lethal_threshold) {
          if (isEdge(cost, width, height, index, 253)) {
            obstacleSet.insert(Obstacle(x, y, index, true, cost_value));
          }
          if (isEdge(cost, width, height, index, 254)) {
            obstacleSet.insert(Obstacle(x, y, index, true, cost_value));
          }
        }
      }

      if (0 <= index - 1 && mark[index - 1] == 0) {
        mark[index - 1] = current + 1;
        marks.push_back(index - 1);
      }
      if (index + 1 < max_size && mark[index + 1] == 0) {
        mark[index + 1] = current + 1;
        marks.push_back(index + 1);
      }
      if (0 <= index - width && mark[index - width] == 0) {
        mark[index - width] = current + 1;
        marks.push_back(index - width);
      }
      if (index + width < max_size && mark[index + width] == 0) {
        mark[index + width] = current + 1;
        marks.push_back(index + width);
      }
    }

    RCLCPP_INFO(logger, "obstacleSet size = %ld", obstacleSet.size());
    obstacles.insert(obstacles.end(), obstacleSet.begin(), obstacleSet.end());

    // find corresponding lethal obstacle
    for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
      if (it->cost != 253) continue;
      double min = 1000;
      Obstacle *nearest;
      for (auto it2 = obstacles.begin(); it2 != obstacles.end(); ++it2) {
        if (it2->cost != 254) continue;
        if (it->is_static != it2->is_static) continue;
        double dist = it2->distance(*it);
        if (dist < min) {
          min = dist;
          nearest = &(*it2);
        }
      }
      it->lethal = nearest;
    }

    // merging nearby obstacle groups
    bool flag = false;
    do {
      flag = false;
      if (groups.size() > 1) {
        for (unsigned long i = 0; i < groups.size()-1; i++) {
          if (groups.back().distance(groups.at(i)) < 5) {
            groups.at(i).combine(groups.back());
            groups.pop_back();
            flag = true;
          }
        }
      }
    } while(flag);
  }

  // 4. create flann index for obstacles (not group obstacles) and point cloud for debug
  unsigned long n = 0;
  unsigned long n_collision = 0;
  for(auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++) {
    n++;
    if (obstacle->in_group) {
      n_collision++;
    }
  }
  if (idx_) {
    delete idx_;
    idx_ = nullptr;
  }
  if (idx_non_collision_) {
    delete idx_non_collision_;
    idx_non_collision_ = nullptr;
  }
  if (data_) {
    delete data_;
    data_ = nullptr;
  }
  if (data_non_collision_) {
    delete data_non_collision_;
    data_non_collision_ = nullptr;
  }
  RCLCPP_INFO(logger, "data length=%ld, data non collision length=%ld", n, n-n_collision);
  if (n == 0 || n-n_collision == 0) {
    return;
  }
  data_ = new cv::Mat(n, 2, CV_32FC1);
  data_non_collision_ = new cv::Mat(n-n_collision, 2, CV_32FC1);
  olist_.clear();
  olist_non_collision_.clear();
  unsigned long i = 0;
  unsigned long j = 0;
  // add index
  
  for (auto oit = obstacles.begin(); oit != obstacles.end(); ++oit) {
    olist_.push_back(*oit);
    data_->at<float>(i, 0) = oit->x;
    data_->at<float>(i, 1) = oit->y;
    i++;
    if (!oit->in_group) {
      olist_non_collision_.push_back(*oit);
      data_non_collision_->at<float>(j, 0) = oit->x;
      data_non_collision_->at<float>(j, 1) = oit->y;
      j++;
    }
  }
  RCLCPP_DEBUG(logger, "making index %ld/%ld", i, n);
  idx_ = new cv::flann::Index(*data_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  RCLCPP_DEBUG(logger, "making index %ld/%ld", j, n-n_collision);
  idx_non_collision_ = new cv::flann::Index(*data_non_collision_, cv::flann::KDTreeIndexParams(2), cvflann::FLANN_DIST_L2);
  RCLCPP_DEBUG(logger, "obstacles = %ld", obstacles.size());
}

std::vector<Obstacle> CaBotPlannerParam::getObstaclesNearPoint(const Point &node, bool collision) const {
  int kdtree_search_radius_in_cells = options.kdtree_search_radius_in_cells;
  int kdtree_max_results = options.kdtree_max_results;

  std::vector<Obstacle> list;
  cv::flann::Index * idx = collision ? idx_non_collision_: idx_;
  if (!idx || !data_) {
    return list;
  }

  cv::Mat query = cv::Mat::zeros(1, 2, CV_32FC1);
  query.at<float>(0) = node.x;
  query.at<float>(1) = node.y;
  std::vector<int> indices;
  std::vector<float> dists;

  int m = idx->radiusSearch(query, indices, dists, kdtree_search_radius_in_cells, kdtree_max_results);
  for (int i = 0; i < m; i++) {
    if (collision) {
      list.push_back(olist_non_collision_[indices[i]]);
    } else {
      list.push_back(olist_[indices[i]]);
    }
  }
  return list;
}
}  // namespace cabot_navigation2