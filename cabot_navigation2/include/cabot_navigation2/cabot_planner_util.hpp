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
#include <opencv2/imgproc.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

namespace cabot_navigation2 {
class CostmapLayerCapture {
 public:
  CostmapLayerCapture(nav2_costmap_2d::LayeredCostmap * layered_costmap, std::string layer_name);
  bool capture();
  nav2_costmap_2d::Costmap2D * getCostmap();

 private:
  nav2_costmap_2d::LayeredCostmap * layered_costmap_;
  std::string layer_name_;
  nav2_costmap_2d::Costmap2D costmap_;
};

class Point {
 public:
  Point();
  Point(float _x, float _y);
  virtual ~Point() = default;
  mutable float x;
  mutable float y;
  void move(float yaw, float length);
  float yaw(float offset = 0);
  float hypot();
  float distance(const Point & other) const;
  Point operator - (const Point & other) const;
  void operator -= (const Point & other);
  Point operator + (const Point & other) const;
  void operator += (const Point & other);
  Point operator * (float s) const;
  void operator *= (float s);
};

class Line {
 public:
  Line();
  Line(Point _s, Point _e);
  Point s;
  Point e;
  Point v;

  double length();
  double dot(Line l);
  double cross(Line l);
  Point closestPoint(Point p);
  Point intersection(Line l);
  tf2::Quaternion quaternion();
  bool intersect_segment(Line l);
  bool segment_intersects_with_line(Line l);
};

class Node: public Point {
 public:
  Node();
  Node(float _x, float _y);
  bool collision = false;
  bool fixed = false;
  double angle = 0;
  Point anchor;
  void reset();
};

class Obstacle;
class Obstacle: public Point {
 public:
  Obstacle();
  Obstacle(float _x, float _y, int _index, bool _is_static = false, int _cost = 0, float _size = 1, bool _in_group = false);
  bool operator < (const Obstacle& rhs ) const;
  float distance(const Point & other) const;
  float getSize() const;
  bool is_static;
  int cost;
  int index;
  float size;
  bool in_group;
  Obstacle *lethal;
};

class ObstacleGroup: public Obstacle {
 public:
  ObstacleGroup();
  bool add(Obstacle obstacle);
  bool complete();
  bool complete(std::vector<cv::Point> &points);
  float getSize(Point & other) const;
  float distance(Point & other) const;
  void combine(ObstacleGroup & other);

  std::set<Obstacle> obstacles_;
  std::vector<cv::Point> hull_;
  bool collision = false;
};

} // namespace cabot_planner
