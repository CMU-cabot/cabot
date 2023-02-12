// Copyright (c) 2020  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CABOT_NAVIGATION2__UTIL_HPP_
#define CABOT_NAVIGATION2__UTIL_HPP_

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace Safety
{
// utility struct and functions
// this could be moved to somewhere else

struct Point
{
  double x;
  double y;
  Point();
  Point(double x_, double y_);
  double distanceTo(Point p);
  Point operator+(Point p);
  Point operator-(Point p);
  Point operator*(double d);
  void transform(tf2::Transform transform);
  geometry_msgs::msg::Point toMsg();
};

struct Line
{
  Point s;
  Point e;
  Point v;
  Line();
  Line(Point s_, Point e_);
  Line(Point o_, tf2::Quaternion q_);
  explicit Line(tf2::Transform t);
  double length();
  double dot(Line l);
  double cross(Line l);
  Point closestPoint(Point p);
  tf2::Quaternion quaternion();
  bool intersect_segment(Line other);
};

}  // namespace Safety

#endif  // CABOT_NAVIGATION2__UTIL_HPP_
