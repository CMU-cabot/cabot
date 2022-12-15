/*******************************************************************************
 * Copyright (c) 2020  Carnegie Mellon University
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
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "cabot/util.hpp"

namespace CaBotSafety
{
// utility struct and functions
// this could be moved to somewhere else

Point::Point() : x(0.0), y(0.0) {}
Point::Point(double x_, double y_) : x(x_),
                                     y(y_)
{
}

double Point::distanceTo(Point p)
{
  return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
}

Point Point::operator+(Point p)
{
  return Point(x + p.x, y + p.y);
}

Point Point::operator-(Point p)
{
  return Point(x - p.x, y - p.y);
}

Point Point::operator*(double d)
{
  return Point(x * d, y * d);
}

void Point::transform(tf2::Transform transform)
{
  tf2::Vector3 v(x, y, 0);
  tf2::Vector3 v2 = transform * v;
  x = v2.x();
  y = v2.y();
}

geometry_msgs::msg::Point Point::toMsg()
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

Line::Line() : s(), e() {}
Line::Line(Point s_, Point e_) : s(s_), e(e_), v(e_ - s_)
{
}
Line::Line(tf2::Transform t)
{
  s = Point(t.getOrigin().x(), t.getOrigin().y());
  auto yaw = tf2::getYaw(t.getRotation());
  e = Point(s.x + cos(yaw), s.y + sin(yaw));
  v = e - s;
}

double Line::length()
{
  return s.distanceTo(e);
}

double Line::dot(Line l)
{
  return v.x * l.v.x + v.y * l.v.y;
}

double Line::cross(Line l)
{
  return v.x * l.v.y - v.y * l.v.x;
}

Point Line::closestPoint(Point p)
{
  Line l(s, p);
  double d = dot(l) / length();
  return s + v * (d / length());
}

tf2::Quaternion Line::quaternion()
{
  tf2::Vector3 v1(1, 0, 0);
  tf2::Vector3 v2(v.x, v.y, 0);

  v1.normalize();
  v2.normalize();

  auto c = v1.cross(v2);
  auto d = v1.dot(v2);
  if (d > (1.0f - 1e-6f))
  {
    return tf2::Quaternion::getIdentity();
  }
  if (d < (1e-6f - 1.0f))
  {
    return tf2::Quaternion::getIdentity().inverse();
  }

  tf2::Quaternion q(c, acos(d));
  auto ret = q.normalized();
  // RCLCPP_INFO(get_logger(), "%.2f - %.2f,%.2f,%.2f,%.2f", d, ret.x(), ret.y(), ret.z(), ret.w());

  return ret;
}

// visualization functions
// this also could be moved to somewhere else
visualization_msgs::msg::MarkerArray array;
int vis_index = 0;


void commit(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub)
{
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.insert(array.markers.begin(), clear_marker);
  vis_pub->publish(array);
  array.markers.clear();
  vis_index = 0;
}

void init_marker(rclcpp::Time now, visualization_msgs::msg::Marker &marker, float r, float g, float b, float a, std::string vis_frame)
{
  marker.header.stamp = now;
  marker.header.frame_id = vis_frame;
  marker.id = vis_index++;
  marker.action = visualization_msgs::msg::Marker::MODIFY;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
}

void add_line(rclcpp::Time now, Line line,
              float size, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker marker;
  init_marker(now, marker, r, g, b, a);
  marker.ns = "people_speed_control_line";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = size;
  marker.points.push_back(line.s.toMsg());
  marker.points.push_back(line.e.toMsg());
  array.markers.push_back(marker);
}

void add_point(rclcpp::Time now, Point point,
               float size, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker marker;
  init_marker(now, marker, r, g, b, a);
  marker.ns = "people_speed_control_point";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  array.markers.push_back(marker);
}

void add_point(rclcpp::Time now, tf2::Transform pose,
               float size, float r, float g, float b, float a)
{
  add_point(now, Point(pose.getOrigin().x(), pose.getOrigin().y()),
            size, r, g, b, a);
}

void add_arrow(rclcpp::Time now, Line line,
               float size, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker marker;
  init_marker(now, marker, r, g, b, a);
  auto q = line.quaternion();
  marker.ns = "people_speed_control_arrow";
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.pose.position.x = line.s.x;
  marker.pose.position.y = line.s.y;
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
  marker.scale.x = line.length();
  marker.scale.y = size;
  marker.scale.z = size;
  array.markers.push_back(marker);
}

void add_text(rclcpp::Time now, std::string text, Point point,
              float size, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker marker;
  init_marker(now, marker, r, g, b, a);
  marker.ns = "people_speed_control_text";
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.pose.position.x = point.x + 1;
  marker.pose.position.y = point.y + 1;
  marker.pose.position.z = 0.5;
  marker.scale.z = size;
  marker.text = text;
  array.markers.push_back(marker);
}

}  // namespace CaBotSafety
