// Copyright (c) 2023  Carnegie Mellon University
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
//
// Speed Visualize Node
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <rclcpp/rclcpp.hpp>
#include <people_msgs/msg/people.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

using std::placeholders::_1;

namespace CaBot
{
class PeopleVisNode : public rclcpp::Node
{
public:
  explicit PeopleVisNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("people_vis_node", options),
    peopleInput_("/people"),
    visOutput_("/people_vis")
  {
    RCLCPP_INFO(get_logger(), "NodeClass Constructor");
    RCLCPP_INFO(get_logger(), "People Visualize Node - %s", __FUNCTION__);

    peopleInput_ = declare_parameter("people_topic", peopleInput_);
    peopleSub = create_subscription<people_msgs::msg::People>(
      peopleInput_, 10, std::bind(&PeopleVisNode::peopleCallback, this, _1));

    visOutput_ = declare_parameter("vis_topic", visOutput_);
    visPub = create_publisher<visualization_msgs::msg::MarkerArray>(visOutput_, 1);
  }

  ~PeopleVisNode()
  {
    RCLCPP_INFO(get_logger(), "NodeClass Destructor");
  }

private:

  void init_marker(visualization_msgs::msg::Marker &marker,
                   people_msgs::msg::Person &person,
                   std::string type_, float r=0.0, float g=0.0, float b=0.0, float a=1.0) {
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "map";
    marker.id = std::stoi(person.name);
    marker.ns = "person_"+type_;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose.position.x = person.position.x;
    marker.pose.position.y = person.position.y;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
  }

  visualization_msgs::msg::Marker get_point(people_msgs::msg::Person &person, float size) {
    visualization_msgs::msg::Marker marker;
    init_marker(marker, person, "point", 0.0, 0.0, 1.0, 1.0);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    return marker;
  }


  visualization_msgs::msg::Marker get_text(people_msgs::msg::Person &person) {
    visualization_msgs::msg::Marker marker;
    init_marker(marker, person, "text", 1.0, 1.0, 1.0);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.z = person.position.z + 0.5;
    marker.scale.z = 0.3;
    marker.text = person.name;
    return marker;
  }


  visualization_msgs::msg::Marker get_arrow(people_msgs::msg::Person &person) {
    visualization_msgs::msg::Marker marker;
    init_marker(marker, person, "arrow");
    marker.type = visualization_msgs::msg::Marker::ARROW;
    float v = sqrt(pow(person.velocity.x, 2)+pow(person.velocity.y, 2));
    float y = atan2(person.velocity.y, person.velocity.x);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, y);
    q=q.normalize();
    
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = v;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    return marker;
  }

  
  void peopleCallback(const people_msgs::msg::People::SharedPtr input)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear_marker);
    
    for (unsigned long i = 0; i < input->people.size(); i++) {
      auto person = input->people[i];
      array.markers.push_back(get_point(person, 0.5));
      array.markers.push_back(get_text(person));
      array.markers.push_back(get_arrow(person));
    }
    visPub->publish(array);
  }

  std::string peopleInput_;
  std::string visOutput_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visPub;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr peopleSub;
};  // class PeopleVisNode

}  // namespace CaBot
//#include <rclcpp_components/register_node_macro.hpp>
//RCLCPP_COMPONENTS_REGISTER_NODE(CaBot::SpeedVisualizeNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaBot::PeopleVisNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
