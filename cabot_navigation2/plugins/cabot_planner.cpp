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

#include <boost/filesystem.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "cabot_navigation2/cabot_planner.hpp"

using namespace std::chrono_literals;

namespace cabot_planner {

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

void Planner::setParam(int width, int height, float origin_x, float origin_y, float resolution, DetourMode detour) {
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  detour_ = detour;
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
  unsigned long i = 0;
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
  unsigned long n = obstacles_.size();
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
  printf("making index %ld/%ld\n", i, n);
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
  float complete_threshold = 0.02;

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
        if (detour_ == DetourMode::RIGHT) {
          yaw = (*n1-*n0).yaw(-M_PI_2);
        } else {
          yaw = (*n1-*n0).yaw(+M_PI_2);
        }
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
        if (detour_ == DetourMode::RIGHT) {
          yaw = (*n1-*n0).yaw(-M_PI_2);
        } else {
          yaw = (*n1-*n0).yaw(+M_PI_2);
        }
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
    if (nodes_[i].x - newNodes[i].x > complete_threshold || 
        nodes_[i].y - newNodes[i].y > complete_threshold) {
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

