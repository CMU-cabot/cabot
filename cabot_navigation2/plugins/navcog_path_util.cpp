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

#include <angles/angles.h>
#include <cabot_navigation2/navcog_path_util.hpp>

namespace cabot_navigation2
{
  rclcpp::Logger util_logger_{rclcpp::get_logger("NavCogPathUtil")};

  nav_msgs::msg::Path normalizedPath(const nav_msgs::msg::Path & path)
  {
    nav_msgs::msg::Path normalized;
    normalized.header = path.header;
    auto temp = path.poses.begin();
    for (auto it = temp + 1; it < path.poses.end(); it++)
    {
      auto prev = tf2::getYaw(temp->pose.orientation);
      auto curr = tf2::getYaw(it->pose.orientation);

      if (fabs(angles::shortest_angular_distance(prev, curr)) < M_PI / 10)
      {
        continue;
      }

      normalized.poses.push_back(*temp);
      temp = it;
    }
    if (normalized.poses.size() == 0 || normalized.poses.back() != *temp)
    {
      normalized.poses.push_back(*temp);
    }
    normalized.poses.push_back(path.poses.back());
    return normalized;
  }


  /* 
   *  @brief estimate path witdh for all poses in the path
   */
  std::vector<PathWidth> estimatePathWidthAndAdjust(nav_msgs::msg::Path &path,
						    nav2_costmap_2d::Costmap2D * costmap,
						    PathEstimateOptions options)
  {
    std::vector<PathWidth> result;

    double r = costmap->getResolution();

    PathWidth estimate{0, 0, 0};
    for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++)
    {
      auto p1 = it;
      auto p2 = it + 1;

      double wx1 = p1->pose.position.x;
      double wy1 = p1->pose.position.y;
      double wx2 = p2->pose.position.x;
      double wy2 = p2->pose.position.y;
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
          auto pw = estimateWidthAt(x, y, yaw, costmap, options);

          if (pw.left < estimate.left)
          {
            estimate.left = std::min(pw.left, std::max(options.path_min_width, dist / 5.0));
          }
          if (pw.right < estimate.right)
          {
            estimate.right = std::min(pw.right, std::max(options.path_min_width, dist / 5.0));
          }
        }
      }

      RCLCPP_INFO(util_logger_, "estimate with left = %.2f, right = %.2f (min %.2f)",
		  estimate.left, estimate.right, options.path_min_width);

      RCLCPP_INFO(util_logger_, "before width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
		  estimate.left, estimate.right, p1->pose.position.x, p1->pose.position.y, p2->pose.position.x, p2->pose.position.y);

      if (estimate.left + estimate.right > options.path_adjusted_minimum_path_width)
      {
	auto adjusted_left = estimate.left;
	auto adjusted_right = estimate.right;

	if (options.path_adjusted_center > 0)
	{
	  adjusted_left = estimate.left * (1 - options.path_adjusted_center);
	  adjusted_right = estimate.right + estimate.left * options.path_adjusted_center;
	}
	else
	{
	  adjusted_right = estimate.right * (1 + options.path_adjusted_center);
	  adjusted_left = estimate.left - estimate.right * options.path_adjusted_center;
	}

	double curr_yaw = tf2::getYaw(it->pose.orientation) + M_PI_2;

	//auto diff_left = adjusted_left - estimate.left;
	auto diff_right = adjusted_right - estimate.right;

	estimate.left = adjusted_left;
	estimate.right = adjusted_right;

	auto curr_diff = diff_right;

	p1->pose.position.x += curr_diff * cos(curr_yaw);
	p1->pose.position.y += curr_diff * sin(curr_yaw);
	p2->pose.position.x += curr_diff * cos(curr_yaw);
	p2->pose.position.y += curr_diff * sin(curr_yaw);

	// if last pose (goal) is updated, publish as updated goal
	if (p2 == path.poses.end() - 1)
	{
	  RCLCPP_INFO(util_logger_, "update goal position");
	}

	RCLCPP_INFO(util_logger_, "after width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
		    estimate.left, estimate.right, p1->pose.position.x, p1->pose.position.y, p2->pose.position.x, p2->pose.position.y);
      }

      estimate.left = std::max(estimate.left, options.path_min_width);
      estimate.right = std::max(estimate.right, options.path_min_width);

      estimate.length = sqrt(pow(p2->pose.position.x - p1->pose.position.x, 2) +
                             pow(p2->pose.position.y - p1->pose.position.y, 2));

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
  PathWidth estimateWidthAt(double x, double y, double yaw,
			    nav2_costmap_2d::Costmap2D * costmap,
			    PathEstimateOptions options)
  {
    double yawl = yaw + M_PI_2;
    double yawr = yaw - M_PI_2;

    double r = costmap->getResolution();
    unsigned char *master = costmap->getCharMap();

    PathWidth pw;

    double robot_size = options.robot_radius + options.safe_margin;
    double dist = options.path_width; // limit by path width
    int N = std::ceil(dist / r);
    double minr = dist;
    // check right side
    for (int i = 0; i <= N; i++)
    {
      unsigned int mx, my;
      double wx, wy;
      wx = x + dist * i / N * cos(yawr);
      wy = y + dist * i / N * sin(yawr);
      costmap->worldToMap(wx, wy, mx, my);
      unsigned int index = costmap->getIndex(mx, my);

      if (master[index] == 254)
      {
        minr = dist * i / N;
        break;
      }
    }
    pw.right = std::max(0.0, std::min(minr, minr - robot_size));

    // check left side
    double minl = dist;
    for (int i = 0; i <= N; i++)
    {
      unsigned int mx, my;
      costmap->worldToMap(x + dist * i / N * cos(yawl), y + dist * i / N * sin(yawl), mx, my);
      unsigned int index = costmap->getIndex(mx, my);
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

  void removeOutlier(std::vector<PathWidth> &estimate)
  {
    PathWidth prev{1.0, 1.0, 1};
    double max_rate = 0.25;
    for (long unsigned int i = 0; i < estimate.size() - 1; i++)
    {
      auto current = estimate[i];
      auto next = estimate[i + 1];
      auto left_rate = (std::abs(current.left * 2 - prev.left - next.left)) / current.length;
      if (max_rate < left_rate)
      {
        if (current.left * 2 < prev.left + next.left)
        {
          current.left = (prev.left + next.right - max_rate * current.length) / 2;
        }
        else
        {
          current.left = (prev.left + next.right + max_rate * current.length) / 2;
        }
      }
      auto right_rate = (std::abs(current.right * 2 - prev.right - next.right)) / current.length;
      if (max_rate < right_rate)
      {
        if (current.right * 2 < prev.right + next.right)
        {
          current.right = (prev.right + next.right - max_rate * current.length) / 2;
        }
        else
        {
          current.right = (prev.right + next.right + max_rate * current.length) / 2;
        }
      }
    }
  }

} // namespace cabot_navigation2

