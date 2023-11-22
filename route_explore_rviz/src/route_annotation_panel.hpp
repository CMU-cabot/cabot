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

#include <rviz/panel.h>
#include <ros/ros.h>
#include <QPushButton>
#include <QListView>
#include <QStringListModel>
#include <QStringList>
#include <OGRE/OgreVector3.h>
#include <tf/tf.h>
#include <route_explore_msgs/Intersection.h>
#include <interactive_markers/interactive_marker_server.h>

namespace rviz
{

class RouteAnnotationPanel: public Panel
{
Q_OBJECT
public:
  explicit RouteAnnotationPanel( QWidget * parent = 0 );
  virtual ~RouteAnnotationPanel();
  void onInitialize() override;
  void load( const rviz::Config& config );
  void save( rviz::Config config ) const;

private Q_SLOTS:
  void addAnnotation();
  void deleteAnnotation();
  void saveAnnotations();
  void currentChanged(const QModelIndex&, const QModelIndex&);
  
private:
  void updateMarkers();
  void loadAnnotations();
  tf::Vector3 getViewPortCenter();
  void addAnnotationControl(tf::Vector3&);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void makeAnnotationMarker(const route_explore_msgs::IntersectionPtr intersection);
  void setControl(visualization_msgs::InteractiveMarker &msg, bool x, bool y, bool r);
    
  QPushButton* add_button_;
  QPushButton* save_button_;
  QPushButton* delete_button_;
  QListView* annotation_list_;
  QItemSelectionModel* selection_model_;
  QStringListModel* annotation_model_;
  int selected_row_;
  std::string selected_;

  std::string configFileName_;
  std::vector<route_explore_msgs::IntersectionPtr> intersections_;
  std::map<std::string, route_explore_msgs::IntersectionPtr> intersectionMap_;
  
  ros::NodeHandle nh_;  
};
} // end namespace rviz
