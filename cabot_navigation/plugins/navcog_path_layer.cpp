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

#include <cabot_navigation/navcog_path_layer.h>
#include <ros/topic.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <chrono>
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(cabot_navigation::NavCogPathLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation
{
  NavCogPathLayer::NavCogPathLayer() : seen_(nullptr),
                                       max_cost_(127),
                                       path_width_(2.0),
                                       path_width_detect_(false),
                                       path_topic_("/path"),
                                       dirty_(false),
                                       walk_weight_({1.0, 0.8, 0.2}),
                                       weight_grid_(0.25)
  {
    costmap_ = nullptr;
  }

  NavCogPathLayer::~NavCogPathLayer()
  {
    delete[] seen_;
  }

  void NavCogPathLayer::activate()
  {
    enabled_ = true;
  }

  void NavCogPathLayer::deactivate()
  {
    enabled_ = false;
  }

  void NavCogPathLayer::reset()
  {
    deactivate();
    resetMaps();
    current_ = true;
    activate();
  }

  void NavCogPathLayer::onInitialize()
  {
    ROS_INFO("NavCogPathLayer::onInitialize, %d", costmap_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    matchSize();
    current_ = true;
    nh.param("path_width", path_width_, path_width_);
    nh.param("path_width_detect", path_width_detect_, path_width_detect_);
    nh.param("path_topic", path_topic_, path_topic_);
    nh.param("walk_weight", walk_weight_, walk_weight_);
    nh.param("weight_grid", weight_grid_, weight_grid_);
    path_sub_ = nh.subscribe(path_topic_, 10, &NavCogPathLayer::pathCallBack, this);
    seen_ = new bool[size_x_ * size_y_];
    ROS_INFO("NavCogPathLayer::onInitialize %d %d", size_x_, size_y_);
  }

  void NavCogPathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                     double *min_y, double *max_x, double *max_y)
  {
    if (dirty_)
    {
      costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
      double ox = costmap->getOriginX();
      double oy = costmap->getOriginY();
      double sx = costmap->getSizeInMetersX();
      double sy = costmap->getSizeInMetersY();
      *min_x = ox;
      *min_y = oy;
      *max_x = sx + ox;
      *max_y = sy + oy;
      dirty_ = false;
    }
  }

  void NavCogPathLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    // copy buffered data
    unsigned char *master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    for (int j = min_j; j < max_j; j++)
    {
      unsigned int it = span * j + min_i;
      for (int i = min_i; i < max_i; i++)
      {
        if (master[it] < 128)
        {
          master[it] = costmap_[it];
        }
        it++;
      }
    }
  }

  void NavCogPathLayer::pathCallBack(const nav_msgs::Path::ConstPtr &path)
  {
    using namespace std::chrono;
    milliseconds s = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    updateWithPath(*path);
    milliseconds e = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    ROS_INFO("NavCogPathLayer::pathCallBack %d ms", e - s);
  }

  /*
    void NavCogPathLayer::update() {
    ros::NodeHandle nh("~/" + name_);
    nav_msgs::Path path = *ros::topic::waitForMessage<nav_msgs::Path>(path_topic_, nh);
    updateWithPath(path);
    }*/

  void NavCogPathLayer::updateWithPath(const nav_msgs::Path &path)
  {
    if (path_.header.stamp.sec != 0 &&
        path_.header.stamp == path.header.stamp)
    {
      return;
    }
    ROS_INFO("path.poses.size() = %lu", path.poses.size());
    ROS_INFO("path_.header.stamp.sec = %d", path_.header.stamp.sec);
    ROS_INFO("path.header.stamp.sec = %d", path.header.stamp.sec);
    path_ = path;
    traversePath(path_);
    dirty_ = true;
  }

  void NavCogPathLayer::traversePath(const nav_msgs::Path &path)
  {
    if (path.poses.empty())
      return;

    // fill the entire costmap with max cost
    memset(costmap_, max_cost_, size_x_ * size_y_ * sizeof(char));

    if (path_width_detect_)
    {
      auto path_width_array = estimatePathWidth(path);
      auto prev_pose = path.poses[0];
      auto prev_width = path_width_array[0];
      for (int i = 0; i < path.poses.size() - 1; i++)
      {
        auto curr_pose = path.poses[i];
        auto curr_width = path_width_array[i];
        auto next_pose = path.poses[i + 1];
        auto next_width = path_width_array[i + 1];

        drawPath(prev_pose.pose, prev_width,
                 curr_pose.pose, curr_width,
                 next_pose.pose, next_width);
        prev_pose = curr_pose;
        prev_width = curr_width;
      }
    }
    else
    {
      auto previous = *path.poses.begin();
      for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++)
      {
        auto current = *(it);
        auto next = *(it + 1);
        drawPath(previous.pose, current.pose, next.pose);
        previous = current;
      }
    }
  }

  /* 
   *  @brief estimate path witdh for all poses in the path
   */
  std::vector<PathWidth> NavCogPathLayer::estimatePathWidth(const nav_msgs::Path &path)
  {
    std::vector<PathWidth> result;
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    double r = costmap->getResolution();

    PathWidth estimate;
    for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++)
    {
      auto p1 = *(it);
      auto p2 = *(it + 1);

      double wx1 = p1.pose.position.x;
      double wy1 = p1.pose.position.y;
      double wx2 = p2.pose.position.x;
      double wy2 = p2.pose.position.y;
      unsigned int mx1, my1, mx2, my2;

      estimate.left = 1000;
      estimate.right = 1000;
      if (costmap->worldToMap(wx1, wy1, mx1, my1) &&
          costmap->worldToMap(wx2, wy2, mx2, my2))
      {
        double dx = wx2 - wx1;
        double dy = wy2 - wy1;
        double yaw = atan2(dy, dx);
        double dist = sqrt(dx * dx + dy * dy);
        int N = std::ceil(dist / r);
        for (int i = 0; i < N; i++)
        {
          double x = wx1 + dx * i / N;
          double y = wy1 + dy * i / N;
          auto pw = estimateWidthAt(x, y, yaw);
          if (pw.left < estimate.left)
          {
            estimate.left = pw.left;
          }
          if (pw.right < estimate.right)
          {
            estimate.right = pw.right;
          }
        }
      }
      result.push_back(estimate);
    }
    // dummy one for the last link
    result.push_back(estimate);
    return result;
  }

  /*
   * @brief estimate path width at (x, y) facing to (yaw) direction. 
   *        raytrace to right and left while it hits to the wall (254)
   */
  PathWidth NavCogPathLayer::estimateWidthAt(double x, double y, double yaw)
  {
    double yawl = yaw + M_PI / 2;
    double yawr = yaw - M_PI / 2;

    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    double r = costmap->getResolution();
    unsigned char *master = costmap->getCharMap();

    PathWidth pw;

    double robot_size = 0.45;
    double dist = path_width_; // limit by path width
    int N = std::ceil(dist / r);
    double minr = dist;
    // check right side
    for (int i = 0; i < N; i++)
    {
      unsigned int mx, my;
      costmap->worldToMap(x + dist * i / N * cos(yawr), y + dist * i / N * sin(yawr), mx, my);
      unsigned int index = getIndex(mx, my);
      if (master[index] == 254)
      {
        minr = dist * i / N;
        break;
      }
    }
    pw.right = std::max(0.0, std::min(minr, minr - robot_size));

    // check left side
    double minl = dist;
    for (int i = 0; i < N; i++)
    {
      unsigned int mx, my;
      costmap->worldToMap(x + dist * i / N * cos(yawl), y + dist * i / N * sin(yawl), mx, my);
      unsigned int index = getIndex(mx, my);
      if (master[index] == 254)
      {
        minl = dist * i / N;
        break;
      }
    }
    pw.left = std::max(0.0, std::min(minl, minl - robot_size));

    return pw;
  }

  double normalized_diff(double a, double b)
  {
    double c = a - b;
    if (M_PI < c)
      c -= 2 * M_PI;
    if (c < -M_PI)
      c += 2 * M_PI;
    return c;
  }

  /* @brief draw path with custom width
   */
  void NavCogPathLayer::drawPath(geometry_msgs::Pose &prev, PathWidth prev_width,
                                 geometry_msgs::Pose &curr, PathWidth curr_width,
                                 geometry_msgs::Pose &next, PathWidth next_width)
  {
    double cost_left = walk_weight_[0];
    double cost_center = walk_weight_[1];
    double cost_right = walk_weight_[2];

    double prev_yaw = tf::getYaw(prev.orientation);
    double curr_yaw = tf::getYaw(curr.orientation);
    double next_yaw = tf::getYaw(next.orientation);
    double curr_diff = normalized_diff(curr_yaw, prev_yaw);
    double next_diff = normalized_diff(next_yaw, curr_yaw);

    // if path is turing, connect differently
    // huristic: inverse of tangent gonna be quiet larger if angle dirrential is small
    double MIN_DIFF = M_PI / 6.0;
    double MAX_DIFF = M_PI * 5.0 / 6.0;
    bool curr_turn = (MIN_DIFF < std::abs(curr_diff) && std::abs(curr_diff) < MAX_DIFF);
    bool next_turn = (MIN_DIFF < std::abs(next_diff) && std::abs(next_diff) < MAX_DIFF);

    double curr_bisector = (M_PI - normalized_diff(curr_yaw, prev_yaw)) / 2;
    double next_bisector = (M_PI - normalized_diff(next_yaw, curr_yaw)) / 2;

    // use bisector if it is almost straight
    double curr_yaw_left = curr_yaw + curr_bisector;
    double curr_yaw_right = curr_yaw + curr_bisector + M_PI;
    double curr_setback_left = 0;
    double curr_setback_right = 0;
    if (curr_turn)
    {
      // use setback if it is turning
      curr_setback_left = prev_width.left / sin(curr_diff) - curr_width.left / tan(curr_diff);
      curr_setback_right = -prev_width.right / sin(curr_diff) + curr_width.right / tan(curr_diff);
      curr_yaw_left = curr_yaw + M_PI / 2;
      curr_yaw_right = curr_yaw - M_PI / 2;
    }

    double next_yaw_left = next_yaw + next_bisector;
    double next_yaw_right = next_yaw + next_bisector + M_PI;
    double next_setback_left = 0;
    double next_setback_right = 0;
    if (next_turn)
    {
      next_setback_left = curr_width.left / sin(next_diff) - next_width.left / tan(next_diff);
      next_setback_right = -curr_width.right / sin(next_diff) + next_width.right / tan(next_diff);
      next_yaw_left = next_yaw + M_PI / 2;
      next_yaw_right = next_yaw - M_PI / 2;
    }

    // calculate start and end points
    double ccx = curr.position.x;
    double ccy = curr.position.y;
    double ncx = next.position.x;
    double ncy = next.position.y;

    double clx = ccx + curr_setback_left * cos(curr_yaw) + curr_width.left * cos(curr_yaw_left);
    double cly = ccy + curr_setback_left * sin(curr_yaw) + curr_width.left * sin(curr_yaw_left);
    double crx = ccx + curr_setback_right * cos(curr_yaw) + curr_width.right * cos(curr_yaw_right);
    double cry = ccy + curr_setback_right * sin(curr_yaw) + curr_width.right * sin(curr_yaw_right);

    double nlx = ncx + next_setback_left * cos(next_yaw) + next_width.left * cos(next_yaw_left);
    double nly = ncy + next_setback_left * sin(next_yaw) + next_width.left * sin(next_yaw_left);
    double nrx = ncx + next_setback_right * cos(next_yaw) + next_width.right * cos(next_yaw_right);
    double nry = ncy + next_setback_right * sin(next_yaw) + next_width.right * sin(next_yaw_right);

    // draw center
    drawPath(ccx, ccy, ncx, ncy, max_cost_ * cost_center);

    const double resolution = layered_costmap_->getCostmap()->getResolution();

    // Left lines
    double dclx = ccx - clx;
    double dcly = ccy - cly;
    double distcl = sqrt(dclx * dclx + dcly * dcly);
    double dnlx = ncx - nlx;
    double dnly = ncy - nly;
    double distnl = sqrt(dnlx * dnlx + dnly * dnly);
    double NL = ceil(std::max(distcl, distnl) / resolution) * 2;
    for (int i = 1; i < NL; i++)
    {
      double sx = (clx * i + ccx * (NL - i)) / NL;
      double sy = (cly * i + ccy * (NL - i)) / NL;
      double ex = (nlx * i + ncx * (NL - i)) / NL;
      double ey = (nly * i + ncy * (NL - i)) / NL;
      int cost = max_cost_ * (cost_left * i + cost_center * (NL - i)) / NL;
      drawPath(sx, sy, ex, ey, cost);
    }

    // Right lines
    double dcrx = ccx - crx;
    double dcry = ccy - cry;
    double distcr = sqrt(dcrx * dcrx + dcry * dcry);
    double dnrx = ncx - nrx;
    double dnry = ncy - nry;
    double distnr = sqrt(dnrx * dnrx + dnry * dnry);
    double NR = ceil(std::max(distcr, distnr) / resolution) * 2;
    for (int i = 1; i < NR; i++)
    {
      double sx = (crx * i + ccx * (NR - i)) / NR;
      double sy = (cry * i + ccy * (NR - i)) / NR;
      double ex = (nrx * i + ncx * (NR - i)) / NR;
      double ey = (nry * i + ncy * (NR - i)) / NR;
      int cost = max_cost_ * (cost_right * i + cost_center * (NR - i)) / NR;
      drawPath(sx, sy, ex, ey, cost);
    }

    /* debug
      ROS_INFO("curr_turn %d next_turn %d", curr_turn, next_turn);
      ROS_INFO("curr_diff %.2f next_diff %.2f", curr_diff, next_diff);
      ROS_INFO("width (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)",
      prev_width.left, prev_width.right,
	       curr_width.left, curr_width.right,
	       next_width.left, next_width.right);
	       ROS_INFO("curr_setback_right %.2f next_setback_right %.2f", curr_setback_right, next_setback_right);
      ROS_INFO("crx %.2f cry %.2f nrx %.2f nry %.2f", crx, cry, nrx, nry);
      ROS_INFO("");
    */
  }

  /* @brief draw path with fixed width
   */
  void NavCogPathLayer::drawPath(geometry_msgs::Pose &prev, geometry_msgs::Pose &curr, geometry_msgs::Pose &next)
  {
    const double half_path_width = path_width_ / 2;
    const double resolution = layered_costmap_->getCostmap()->getResolution();
    double stride_length = 0.5 * resolution;
    const int strides_count = static_cast<int>(ceil(half_path_width / stride_length));
    const int windows = static_cast<int>(ceil(half_path_width / weight_grid_));
    const int strides_per_window = static_cast<int>(ceil(weight_grid_ / stride_length));
    double left = walk_weight_[0];
    double center = walk_weight_[1];
    double right = walk_weight_[2];

    auto prev_yaw = tf::getYaw(prev.orientation);
    auto curr_yaw = tf::getYaw(curr.orientation);
    auto next_yaw = tf::getYaw(next.orientation);

    auto curr_bisector = (M_PI - (curr_yaw - prev_yaw)) / 2;
    auto next_bisector = (M_PI - (next_yaw - curr_yaw)) / 2;

    auto curr_yaw_left = curr_yaw + curr_bisector;
    auto curr_yaw_right = curr_yaw + curr_bisector + M_PI;
    auto curr_width = half_path_width / std::sin(curr_bisector);

    auto next_yaw_left = next_yaw + next_bisector;
    auto next_yaw_right = next_yaw + next_bisector + M_PI;
    auto next_width = half_path_width / std::sin(next_bisector);

    // Draw the center line first
    drawPath(curr.position.x, curr.position.y, next.position.x, next.position.y, max_cost_ * center);
    for (auto i = 0; i < strides_count; i++)
    {
      double step_cost_ratio = (1.0 / (windows - 1)) * (i / strides_per_window);
      double curr_length = stride_length * (i + 1) * curr_width;
      double next_length = stride_length * (i + 1) * next_width;

      // Left lines
      drawPath(curr.position.x + curr_length * cos(curr_yaw_left),
               curr.position.y + curr_length * sin(curr_yaw_left),
               next.position.x + next_length * cos(next_yaw_left),
               next.position.y + next_length * sin(next_yaw_left),
               max_cost_ * (center * (1.0 - step_cost_ratio) + left * step_cost_ratio));

      // Right lines
      drawPath(curr.position.x + curr_length * cos(curr_yaw_right),
               curr.position.y + curr_length * sin(curr_yaw_right),
               next.position.x + next_length * cos(next_yaw_right),
               next.position.y + next_length * sin(next_yaw_right),
               max_cost_ * (center * (1.0 - step_cost_ratio) + right * step_cost_ratio));
    }
  }

  void NavCogPathLayer::drawPath(double wx1, double wy1, double wx2, double wy2, int cost = 63)
  {
    unsigned int mx1, my1;
    unsigned int mx2, my2;
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    double r = costmap->getResolution();
    if (costmap->worldToMap(wx1, wy1, mx1, my1) &&
        costmap->worldToMap(wx2, wy2, mx2, my2))
    {
      double dx = wx2 - wx1;
      double dy = wy2 - wy1;
      double dist = sqrt(dx * dx + dy * dy);
      int N = std::ceil(dist / r);
      for (int i = 0; i < N; i++)
      {
        unsigned int mx, my;
        costmap->worldToMap(wx1 + dx * i / N, wy1 + dy * i / N, mx, my);
        unsigned int index = getIndex(mx, my);
        costmap_[index] = cost;
      }
    }
  }

} // namespace cabot_navigation
