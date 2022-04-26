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

namespace cabot_planner {

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

} // namespace cabot_planner
