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

#include "cabot_navigation2/cabot_planner_util.hpp"

namespace cabot_navigation2 {

// MARK: CosmapLayerCapture
CostmapLayerCapture::CostmapLayerCapture(nav2_costmap_2d::LayeredCostmap *layered_costmap, std::string layer_name) {
  layered_costmap_ = layered_costmap;
  layer_name_ = layer_name;
}

bool CostmapLayerCapture::capture() {
  auto width = layered_costmap_->getCostmap()->getSizeInCellsX();
  auto height = layered_costmap_->getCostmap()->getSizeInCellsY();
  auto resolution = layered_costmap_->getCostmap()->getResolution();
  auto origin_x = layered_costmap_->getCostmap()->getOriginX();
  auto origin_y = layered_costmap_->getCostmap()->getOriginY();
  costmap_.resizeMap(width, height, resolution, origin_x, origin_y);

  auto plugins = layered_costmap_->getPlugins();
  bool flag = false;
  for (auto plugin = plugins->begin(); plugin != plugins->end(); plugin++) {
    if (plugin->get()->getName() == layer_name_) {
      // double min_x, min_y, max_x, max_y;
      // plugin->get()->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
      // printf("%.2f %.2f %.2f %.2f\n", min_x, min_y, max_x, max_y);
      plugin->get()->updateCosts(costmap_, 0, 0, width, height);
    }
    if (plugin->get()->getName() == "inflation_layer") {
      plugin->get()->updateCosts(costmap_, 0, 0, width, height);
    }
  }
  return flag;
}

nav2_costmap_2d::Costmap2D *CostmapLayerCapture::getCostmap() { return &costmap_; }

Point::Point(){}
Point::Point(float _x, float _y) {
  x = _x;
  y = _y;
}

void Point::move(float yaw, float length) {
  x += std::cos(yaw) * length;
  y += std::sin(yaw) * length;
}
float Point::yaw(float offset) {
  float yaw = std::atan2(y, x) + offset;
  if (yaw > M_PI) {
    yaw -= M_PI * 2;
  }
  if (yaw < -M_PI) {
    yaw += M_PI * 2;
  }
  return yaw;
}
float Point::hypot() { return std::hypot(x, y); }
float Point::distance(const Point &other) const { return std::hypot(x - other.x, y - other.y); }
Point Point::operator-(const Point &other) const { return Point(x - other.x, y - other.y); }
void Point::operator-=(const Point &other) {
  x -= other.x;
  y -= other.y;
}
Point Point::operator+(const Point &other) const { return Point(x + other.x, y + other.y); }
void Point::operator+=(const Point &other) {
  x += other.x;
  y += other.y;
}
Point Point::operator*(float s) const { return Point(x * s, y * s); }
void Point::operator*=(float s) {
  x *= s;
  y *= s;
}

Line::Line() {
  s = Point(0, 0);
  e = Point(0, 0);
  v = Point(0, 0);
}
Line::Line(Point _s, Point _e) {
  s = _s;
  e = _e;
  v = e - s;
}

double Line::length() { return v.hypot(); }

double Line::dot(Line l) { return v.x * l.v.x + v.y * l.v.y; }

double Line::cross(Line l) {
  double c = v.x * l.v.y - v.y * l.v.x;
  return c;
}

Point Line::closestPoint(Point p) {
  Line l(s, p);
  double d = dot(l) / length();
  if (d < length()) {
    return s + v * (d / length());
  } else {
    return s + v;
  }
}

Point Line::intersection(Line l) {
  Point &p1 = s;
  Point &p2 = e;
  Point &p3 = l.s;
  Point &p4 = l.e;
  auto det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  auto t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  auto x = t * p1.x + (1.0 - t) * p2.x;
  auto y = t * p1.y + (1.0 - t) * p2.y;
  return Point(x, y);
}

tf2::Quaternion Line::quaternion() {
  tf2::Vector3 v1(1, 0, 0);
  tf2::Vector3 v2(v.x, v.y, 0);

  v1.normalize();
  v2.normalize();

  auto c = v1.cross(v2);
  auto d = v1.dot(v2);
  if (d > (1.0f - 1e-6f)) {
    return tf2::Quaternion::getIdentity();
  }
  if (d < (1e-6f - 1.0f)) {
    return tf2::Quaternion::getIdentity().inverse();
  }

  tf2::Quaternion q(c, acos(d));
  auto ret = q.normalized();
  return ret;
}

bool Line::intersect_segment(Line l) {
  auto l1 = Line(s, l.s);
  auto l2 = Line(s, l.e);
  auto l3 = Line(l.s, s);
  auto l4 = Line(l.s, e);
  return cross(l1) * cross(l2) < 0 && l.cross(l3) * l.cross(l4) < 0;
}

bool Line::segment_intersects_with_line(Line l) {
  auto l3 = Line(l.s, s);
  auto l4 = Line(l.s, e);

  // need to deal with the cases cross returns 0 (perpendicular)
  return (l.cross(l3) <= 0 && l.cross(l4) > 0) || (l.cross(l3) < 0 && l.cross(l4) >= 0);
}

Node::Node() : Point() {
  anchor.x = x;
  anchor.y = y;
}
Node::Node(float _x, float _y) : Point(_x, _y) {
  anchor.x = x;
  anchor.y = y;
}
void Node::reset() {
  x = anchor.x;
  y = anchor.y;
}

Obstacle::Obstacle() : Point() {}
Obstacle::Obstacle(float _x, float _y, int _index, bool _is_static, int _cost, float _size, bool _in_group): Point(_x, _y) {
  index = _index;
  is_static = _is_static;
  cost = _cost;
  size = _size;
  in_group = _in_group;
  lethal = nullptr;
}

bool Obstacle::operator<(const Obstacle &rhs) const { return index < rhs.index; }

float Obstacle::distance(const Point &other) const {
  float dist = std::hypot(x - other.x, y - other.y);
  if (lethal == nullptr) return dist;

  auto l1 = Line(*lethal, other);
  auto l2 = Line(*lethal, *this);

  dist = l1.length();
  float size = l2.length();

  auto dot = l1.dot(l2);
  dist = dot / l2.length();
  return dist;
}

float Obstacle::getSize() const {
  if (lethal == nullptr) return size;
  auto l2 = Line(*lethal, *this);
  return l2.length();
}

ObstacleGroup::ObstacleGroup() : Obstacle() {}
bool ObstacleGroup::add(Obstacle obstacle) {
  auto result = obstacles_.insert(obstacle);
  return result.second;
}
bool ObstacleGroup::complete() {
  std::vector<cv::Point> points;
  for (auto it = obstacles_.begin(); it != obstacles_.end(); it++) {
    points.push_back(cv::Point(it->x, it->y));
  }
  return complete(points);
}
bool ObstacleGroup::complete(std::vector<cv::Point> &points) {
  try {
    cv::convexHull(points, hull_);

    cv::Point2f sum = std::accumulate(hull_.begin(), hull_.end(), cv::Point2f(0.0f, 0.0f), std::plus<cv::Point2f>());
    x = sum.x / hull_.size();
    y = sum.y / hull_.size();
    return true;
  } catch (std::exception &e) {
    return false;
  }
}

float ObstacleGroup::getSize(Point &other) const {
  auto l1 = Line(*this, other);
  for (unsigned long i = 0; i < hull_.size(); i++) {
    auto p0 = hull_.at(i);
    auto p1 = hull_.at((i + 1) % hull_.size());
    auto l2 = Line(Point(p0.x, p0.y), Point(p1.x, p1.y));
    if (l2.segment_intersects_with_line(l1)) {
      auto cp = l1.intersection(l2);
      auto d = this->distance(cp);
      return d;
    }
  }
  return 0;
}

float ObstacleGroup::distance(Point &other) const {
  if (ObstacleGroup *v = dynamic_cast<ObstacleGroup *>(&other)) {
    if (const Point *p = dynamic_cast<const Point *>(this)) {
      auto dist = std::hypot(x - other.x, y - other.y);
      auto size1 = getSize(other);
      auto point = *p;
      auto size2 = v->getSize(point);
      return dist - size1 - size2;
    }
  }
  return std::hypot(x - other.x, y - other.y);
}

void ObstacleGroup::combine(ObstacleGroup &other) {
  std::vector<cv::Point> temp;
  temp.insert(temp.end(), hull_.begin(), hull_.end());
  temp.insert(temp.end(), other.hull_.begin(), other.hull_.end());
  complete(temp);
  collision = collision || other.collision;

  obstacles_.insert(other.obstacles_.begin(), other.obstacles_.end());
}

}  // namespace cabot_navigation2
