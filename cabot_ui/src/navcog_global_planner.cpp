// Copyright (c) 2020 Carnegie Mellon University
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

#include <pluginlib/class_list_macros.h>
#include "nav_msgs/Path.h"
#include "navcog_global_planner.h"


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navcog_global_planner::NavCogGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace navcog_global_planner {

  static nav_msgs::Path _path;
  static ros::NodeHandle nh;
  
  void pathCallback(const nav_msgs::Path& path) {
    std::cout << path << std::endl;
    _path = path;
  }
    
  NavCogGlobalPlanner::NavCogGlobalPlanner (){
  }

  NavCogGlobalPlanner::NavCogGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    std::cout << "init NavCogGlobalPlanner" << std::endl;
    initialize(name, costmap_ros);
  }


  void NavCogGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    //ros::Subscriber sub = nh.subscribe("path", 1000, pathCallback);
  }

  bool NavCogGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    std::cout << "makePlan NavCogGlobalPlanner" << std::endl;

    nav_msgs::Path _path;

    _path = *ros::topic::waitForMessage<nav_msgs::Path>("path", nh);

    for(int i = 0; i < _path.poses.size(); i++){
      std::cout << i << "  --------------------" << std::endl;
      std::cout << _path.poses.at(i) << std::endl;
      plan.push_back(_path.poses.at(i));
    }
    
    return true;
  }
};
