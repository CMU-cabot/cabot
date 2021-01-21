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

#ifndef _NAVCOG_PATH_LAYER_H_
#define _NAVCOG_PATH_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <deque>

namespace cabot_navigation
{
    struct PathWidth
    {
        double left;
        double right;
    };

    class NavCogPathLayer : public costmap_2d::CostmapLayer
    {
    public:
        NavCogPathLayer();

        ~NavCogPathLayer() override;

        void onInitialize() override;

        void updateBounds(double robot_x, double robot_y, double robot_yaw,
                          double *min_x, double *min_y,
                          double *max_x, double *max_y) override;

        void updateCosts(costmap_2d::Costmap2D &master_grid,
                         int min_i, int min_j, int max_i, int max_j) override;

        void activate() override;

        void deactivate() override;

        void reset() override;

        void pathCallBack(const nav_msgs::Path::ConstPtr &path);

    private:
        void updateWithPath(const nav_msgs::Path &path);

        void traversePath(const nav_msgs::Path &path);

        std::vector<PathWidth> estimatePathWidth(const nav_msgs::Path &path);
        PathWidth estimateWidthAt(double x, double y, double yaw);

        void drawPath(geometry_msgs::Pose &p1, PathWidth w1,
                      geometry_msgs::Pose &p2, PathWidth w2,
                      geometry_msgs::Pose &p3, PathWidth w3);

        void drawPath(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2, geometry_msgs::Pose &p3);

        void drawPath(double wx1, double wy1, double wx2, double wy2, int cost);

        double max_cost_;
        double path_width_;
        bool path_width_detect_;
        bool *seen_;
        bool dirty_;
        std::vector<double> walk_weight_;
        double weight_grid_;

        nav_msgs::Path path_;
        std::string path_topic_;

        ros::Subscriber path_sub_;

        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_{};
        double previous_path_width_ = path_width_ / 2;
    };
} // namespace cabot_navigation
#endif
