// Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <route_explore_msgs/Intersection.h>

#include <yaml-cpp/yaml.h>

template <class T>
YAML::Node to_yaml(std::vector<T>&);
YAML::Node to_yaml(route_explore_msgs::Intersection&);
YAML::Node to_yaml(route_explore_msgs::Way&);
YAML::Node to_yaml(std_msgs::Header&);
YAML::Node to_yaml(geometry_msgs::Pose&);
YAML::Node to_yaml(geometry_msgs::Point&);
YAML::Node to_yaml(geometry_msgs::Quaternion&);

std::vector<route_explore_msgs::IntersectionPtr> parseIntersections(YAML::Node);
bool parseIntersection(YAML::Node, route_explore_msgs::Intersection&);
bool parseWay(YAML::Node, route_explore_msgs::Way&);
bool parseHeader(YAML::Node, std_msgs::Header&);
bool parsePose(YAML::Node, geometry_msgs::Pose&);
bool parsePoint(YAML::Node, geometry_msgs::Point&);
bool parseQuaternion(YAML::Node, geometry_msgs::Quaternion&);

template <class T>
YAML::Node to_yaml(std::vector<T>& alist) {
  YAML::Node list;
  for(auto it = alist.begin(); it < alist.end(); it++) {
    list.push_back(to_yaml(*it));
  }
  return list;
}

YAML::Node to_yaml(route_explore_msgs::Intersection& intersection) {
  YAML::Node yaml;
  yaml["header"] = to_yaml(intersection.header);
  yaml["name"] = intersection.name;
  yaml["position"] = to_yaml(intersection.position);
  yaml["ways"] = to_yaml(intersection.ways);
  return yaml;
}

YAML::Node to_yaml(route_explore_msgs::Way& way) {
  YAML::Node yaml;
  yaml["name"] = way.name;
  yaml["pose"] = to_yaml(way.pose);
  return yaml;
}

YAML::Node to_yaml(std_msgs::Header& header) {
  YAML::Node yaml;
  yaml["seq"] = header.seq;
  YAML::Node stamp;
  stamp["sec"] = header.stamp.sec;
  stamp["nsec"] = header.stamp.nsec;
  yaml["stamp"] = stamp;
  yaml["frame_id"] = header.frame_id;
  return yaml;
}

YAML::Node to_yaml(geometry_msgs::Pose& pose) {
  YAML::Node yaml;
  yaml["position"] = to_yaml(pose.position);
  yaml["orientation"] = to_yaml(pose.orientation);
  return yaml;
}

YAML::Node to_yaml(geometry_msgs::Point& point) {
  YAML::Node yaml;
  yaml["x"] = point.x;
  yaml["y"] = point.y;
  yaml["z"] = point.z;
  return yaml;
}

YAML::Node to_yaml(geometry_msgs::Quaternion& quat) {
  YAML::Node yaml;
  yaml["x"] = quat.x;
  yaml["y"] = quat.y;
  yaml["z"] = quat.z;
  yaml["w"] = quat.w;
  return yaml;
}

std::vector<route_explore_msgs::IntersectionPtr> parseIntersections(YAML::Node yaml) {
  std::vector<route_explore_msgs::IntersectionPtr> intersections_msg;
  auto intersections = yaml["intersections"];
  if (intersections) {
    for (std::size_t i=0; i<intersections.size(); i++) {
      auto intersection = boost::make_shared<route_explore_msgs::Intersection>();
      
      if (parseIntersection(intersections[i], *intersection)) {
	intersections_msg.push_back(intersection);
      }
    }      
  } else {
    ROS_ERROR("there is no intersections");
  }
  return intersections_msg;
}

bool parseIntersection(YAML::Node yaml , route_explore_msgs::Intersection& intersection) {
  auto header = yaml["header"];
  auto name = yaml["name"];
  auto position = yaml["position"];
  auto ways = yaml["ways"];
  
  if (!header && !name && !position && !ways) {
    return false;
  }

  if (!parseHeader(header, intersection.header)) {
    return false;
  }

  intersection.name = name.as<std::string>();
  
  if (!parsePoint(position, intersection.position)) {
    return false;
  }
  
  for(std::size_t i=0; i < ways.size(); i++) {
    route_explore_msgs::Way way;
    if (!parseWay(ways[i], way)) {
      return false;
    }
    intersection.ways.push_back(way);
  }
  
  return true;
}

bool parseHeader(YAML::Node yaml, std_msgs::Header& header)
{
  auto seq = yaml["seq"];
  auto stamp = yaml["stamp"];
  auto frame_id = yaml["frame_id"];
  if (!seq || !stamp || !frame_id) {
    return false;
  }

  header.seq = seq.as<unsigned int>();

  auto sec = stamp["sec"];
  auto nsec = stamp["nsec"];
  if (!sec || !nsec) {
    return false;
  }
  header.stamp.sec = sec.as<unsigned int>();
  header.stamp.nsec = nsec.as<unsigned int>();

  header.frame_id = frame_id.as<std::string>();

  return true;
}

bool parseWay(YAML::Node yaml, route_explore_msgs::Way& way)
{
  auto name = yaml["name"];
  auto pose = yaml["pose"];
  if (!name || !pose) {
    return false;
  }
  way.name = name.as<std::string>();

  if (!parsePose(pose, way.pose)) {
    return false;
  }
  return true;
}

bool parsePose(YAML::Node yaml, geometry_msgs::Pose& pose)
{
  auto position = yaml["position"];
  auto orientation = yaml["orientation"];
  if (!position || !orientation) {
    return false;
  }
  if (!parsePoint(position, pose.position)) {
    return false;
  }
  if (!parseQuaternion(orientation, pose.orientation)) {
    return false;
  }
  return true;
}

bool parsePoint(YAML::Node yaml, geometry_msgs::Point& point)
{
  auto x = yaml["x"];
  auto y = yaml["y"];
  auto z = yaml["z"];
  if (!x || !y || !z) {
    return false;
  }
  point.x = x.as<double>();
  point.y = y.as<double>();
  point.z = z.as<double>();
  return true;
}

bool parseQuaternion(YAML::Node yaml, geometry_msgs::Quaternion& quat)
{
  auto x = yaml["x"];
  auto y = yaml["y"];
  auto z = yaml["z"];
  auto w = yaml["w"];
  if (!x || !y || !z || !w) {
    return false;
  }
  quat.x = x.as<double>();
  quat.y = y.as<double>();
  quat.z = z.as<double>();
  quat.w = w.as<double>();
  return true;
}


