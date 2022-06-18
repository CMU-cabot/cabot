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

#include <math.h>
#include <nav2_costmap_2d/layered_costmap.hpp>

namespace cabot_navigation2 {
class CostmapLayerCapture {
 public:
  CostmapLayerCapture(nav2_costmap_2d::LayeredCostmap * layered_costmap, std::string layer_name) {
    layered_costmap_ = layered_costmap;
    layer_name_ = layer_name;
  }

  bool capture() {
    auto width = layered_costmap_->getCostmap()->getSizeInCellsX();
    auto height = layered_costmap_->getCostmap()->getSizeInCellsY();
    auto resolution = layered_costmap_->getCostmap()->getResolution();
    auto origin_x = layered_costmap_->getCostmap()->getOriginX();
    auto origin_y = layered_costmap_->getCostmap()->getOriginY();
    costmap_.resizeMap(width, height, resolution, origin_x, origin_y);

    auto plugins = layered_costmap_->getPlugins();
    bool flag = false;
    for(auto plugin = plugins->begin(); plugin != plugins->end(); plugin++) {
      if (plugin->get()->getName() == layer_name_) {
        //double min_x, min_y, max_x, max_y;
        //plugin->get()->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
        //printf("%.2f %.2f %.2f %.2f\n", min_x, min_y, max_x, max_y);
        plugin->get()->updateCosts(costmap_, 0, 0, width, height);
      }
      if (plugin->get()->getName() == "inflation_layer") {
        plugin->get()->updateCosts(costmap_, 0, 0, width, height);
      }
    }
    return flag;
  }

  nav2_costmap_2d::Costmap2D * getCostmap() {
    return &costmap_;
  }
 private:
  nav2_costmap_2d::LayeredCostmap * layered_costmap_;
  std::string layer_name_;
  nav2_costmap_2d::Costmap2D costmap_;
};

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
  bool fixed = false;
  double angle = 0;
  Point anchor;
  void reset() {
    x = anchor.x;
    y = anchor.y;
  }
};

class Obstacle: public Point {
 public:
  Obstacle(): Point() { }
  Obstacle(float _x, float _y, int _index, float _size, bool _in_group = false): Point(_x, _y) { index = _index; size = _size; in_group = _in_group; }

  bool operator < (const Obstacle& rhs ) const {
    return index < rhs.index;
  }

  int index;
  mutable float size;
  mutable bool in_group;
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

    for(it = obstacles_.begin(); it != obstacles_.end(); it++) {
      float yaw = std::atan2(it->y - y, it->x - x);
      int deg = degree(yaw);
      float dist = std::hypot(it->y - y, it->x - x);
      if (distance_map[deg] < dist) {
        distance_map[deg] = dist;
      }
    }
  }
  int degree(float yaw) const {
    if (yaw < 0) {
      yaw += M_PI*2;
    }
    int deg = (int)std::floor(yaw / M_PI * 180);
    deg = deg * MAPSIZE / 360;
    assert(deg >= 0);
    assert(deg < MAPSIZE);
    return deg;
  }
  float getSize(Point & other) const {
    float yaw = std::atan2(other.y - y, other.x - x);
    int deg = degree(yaw);
    return distance_map[deg];
  }
  std::set<Obstacle> obstacles_;
  int MAPSIZE = 30;
  float distance_map[30] = {};
};

} // namespace cabot_planner
