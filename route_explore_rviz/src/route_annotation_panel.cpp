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

#include "route_annotation_panel.hpp"
#include "msg_parser.hpp"

#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>
#include <ros/ros.h>
#include <math.h>

#include <rviz/visualization_manager.h>
#include <interactive_markers/menu_handler.h>

#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <OGRE/OgreRenderWindow.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <route_explore_msgs/Intersection.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>

using namespace visualization_msgs;

namespace rviz
{
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  interactive_markers::MenuHandler menu_handler;

  
  RouteAnnotationPanel::RouteAnnotationPanel( QWidget* parent )
    : Panel( parent )
  {
    // init components
    add_button_ = new QPushButton( "Add annotation" );
    save_button_ = new QPushButton( "Save annotations" );
    delete_button_ = new QPushButton( "Delete annotation" );

    annotation_list_ = new QListView();
    annotation_model_ = new QStringListModel(this);
    annotation_list_->setModel(annotation_model_);
    selection_model_ = annotation_list_->selectionModel();
  
    auto label = new QLabel("Annotations:");
    label->setAlignment(Qt::AlignVCenter);

    // layout
    QVBoxLayout* vlayout = new QVBoxLayout;
    QHBoxLayout* hlayout = new QHBoxLayout;

    hlayout->addWidget(add_button_);
    hlayout->addWidget(save_button_);
    
    vlayout->addLayout(hlayout);
    vlayout->addWidget(label);
    vlayout->addWidget(annotation_list_);
    vlayout->addWidget(delete_button_);
    
    setLayout( vlayout );

    QObject::connect( add_button_, SIGNAL(clicked()), this, SLOT(addAnnotation()));
    QObject::connect( save_button_, SIGNAL(clicked()), this, SLOT(saveAnnotations()));
    QObject::connect( delete_button_, SIGNAL(clicked()), this, SLOT(deleteAnnotation()));
    QObject::connect( selection_model_, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this, SLOT(currentChanged(QModelIndex, QModelIndex)));
  }

  RouteAnnotationPanel::~RouteAnnotationPanel()
  {
  }

  void RouteAnnotationPanel::onInitialize()
  {
    server.reset( new interactive_markers::InteractiveMarkerServer("route_annotation_controls","",false) );

    menu_handler.insert("Increase Way", boost::bind(&RouteAnnotationPanel::processFeedback, this, _1));
    menu_handler.insert("Decrease Way", boost::bind(&RouteAnnotationPanel::processFeedback, this, _1));

    nh_.getParam("rviz/annotation_file", configFileName_);

    selected_row_ = -1;

    loadAnnotations();
    updateMarkers();
  }

// private Q_SLOTS

  void RouteAnnotationPanel::saveAnnotations()
  {
    YAML::Node yaml;
    YAML::Node list;
    for(auto it = intersections_.begin(); it < intersections_.end(); it++) {
      list.push_back(to_yaml(**it));
    }
    yaml["intersections"] = list;
    YAML::Emitter out;
    out << yaml;
    ROS_INFO("Save file\n%s", out.c_str());

    std::ofstream fout(configFileName_);
    fout << yaml;
  }

  void RouteAnnotationPanel::addAnnotation()
  {
    auto center = getViewPortCenter();
    addAnnotationControl(center);
  }

  void RouteAnnotationPanel::deleteAnnotation()
  {
    if (selected_row_ >= 0 && selected_row_ < intersections_.size()) {
      ROS_INFO("delete selected row %d", selected_row_);
      intersections_.erase(intersections_.begin() + selected_row_);
      updateMarkers();
    }
  }

  void RouteAnnotationPanel::currentChanged(const QModelIndex& current, const QModelIndex& prev) {
    ROS_INFO("current=%d, prev=%d", current.row(), prev.row());
    selected_row_ = current.row();
  }

// private functions
  void RouteAnnotationPanel::loadAnnotations()
  {
    try {
      YAML::Node config = YAML::LoadFile(configFileName_);
      intersections_ = parseIntersections(config);

      ROS_INFO("loaded %ld intersections", intersections_.size());
    } catch (std::exception &e) {
      ROS_INFO("file not found");
    }
  }

  void RouteAnnotationPanel::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    auto found_intersection = intersectionMap_.find(feedback->marker_name);
    
    if( feedback->mouse_point_valid )
    {
      mouse_point_ss << " at " << feedback->mouse_point.x
		     << ", " << feedback->mouse_point.y
		     << ", " << feedback->mouse_point.z
		     << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      if (found_intersection != intersectionMap_.end()) {
	auto intersection = found_intersection->second;
	if (feedback->menu_entry_id == 1) {
	  route_explore_msgs::Way way;
	  std::ostringstream name;  
	  name << intersection->name << "_way_" << intersection->ways.size();
	  way.name = name.str();
	  auto rad = M_PI / 2 * intersection->ways.size();
	  
	  tf2::Quaternion q;
	  q.setRPY(0, 0, rad);
	  
	  way.pose.position.x = intersection->position.x + cos(rad);
	  way.pose.position.y = intersection->position.y + sin(rad);
	  tf2::convert(q, way.pose.orientation);
	  
	  intersection->ways.push_back(way);
	} else {
	  if (intersection->ways.size() > 0) {
	    intersection->ways.pop_back();
	    ROS_INFO("removed");
	  }
	}
	updateMarkers();
	
	ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      }
      
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      if (found_intersection != intersectionMap_.end()) {
	auto intersection = found_intersection->second;

	// update intersection
	if (intersection->name == feedback->marker_name) {
	  auto dx = feedback->pose.position.x - intersection->position.x;
	  auto dy = feedback->pose.position.y - intersection->position.y;
	  
	  intersection->position.x += dx;
	  intersection->position.y += dy;
	  
	  for (auto it = intersection->ways.begin(); it < intersection->ways.end(); it++) {
	    (*it).pose.position.x += dx;
	    (*it).pose.position.y += dy;
	  }
	}
	else {
	  for (auto it = intersection->ways.begin(); it < intersection->ways.end(); it++) {
	    if ((*it).name == feedback->marker_name) {
	      (*it).pose = feedback->pose;
	    }
	  }
	}
	updateMarkers();
	
      }
      break;
      
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      //ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      if (found_intersection != intersectionMap_.end()) {
	ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
	selected_ = feedback->marker_name;
      } else {
	selected_ = "";
      }
      updateMarkers();
      break;
    }

    server->applyChanges();
  }

  tf::Vector3 RouteAnnotationPanel::getViewPortCenter() {
    auto viewport = vis_manager_->getViewManager()->getRenderPanel()->getRenderWindow()->getViewport(0);
    auto ray = viewport->getCamera()->getCameraToViewportRay(0.5, 0.5);
    Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0);
    auto result = ray.intersects(plane);
    if (result.first) {
      auto point = ray.getPoint(result.second);
      ROS_INFO("%.2f %.2f %.2f", point.x, point.y, point.z);
      return tf::Vector3({point.x, point.y, point.z});

    }
    ROS_INFO("cannot find intersection");
    return tf::Vector3({0,0,0});
  }

  Marker makeArrow( InteractiveMarker &msg )
  {
    Marker marker;
    
    marker.type = Marker::ARROW;    
    marker.scale.x = msg.scale * 1.0;
    marker.scale.y = msg.scale * 0.2;
    marker.scale.z = msg.scale * 0.2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    return marker;
  }
  
  InteractiveMarkerControl& makeArrowControl( InteractiveMarker &msg )
  {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    
    control.markers.push_back( makeArrow(msg) );
    msg.controls.push_back( control );
    
    return msg.controls.back();
  }

  Marker makePoint( InteractiveMarker &msg )
  {
    Marker marker;
    
    marker.type = Marker::SPHERE;    
    marker.scale.x = msg.scale * 0.5;
    marker.scale.y = msg.scale * 0.5;
    marker.scale.z = msg.scale * 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    return marker;
  }
  
  InteractiveMarkerControl& makePointControl( InteractiveMarker &msg )
  {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    
    control.markers.push_back( makePoint(msg) );
    msg.controls.push_back( control );
    
    return msg.controls.back();
  }

  void RouteAnnotationPanel::setControl(InteractiveMarker &msg, bool x, bool y, bool r)
  {
    InteractiveMarkerControl control;

    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    if (x) {
      orien.normalize();
      tf::quaternionTFToMsg(orien, control.orientation);
      control.name = "move_x";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      msg.controls.push_back(control);
    }

    if (y) {
      orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
      orien.normalize();
      tf::quaternionTFToMsg(orien, control.orientation);
      control.name = "move_y";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      msg.controls.push_back(control);
    }

    if (r) {
      orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
      orien.normalize();
      tf::quaternionTFToMsg(orien, control.orientation);
      control.name = "rotate_z";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      msg.controls.push_back(control);
    }
  
  }
  
  void RouteAnnotationPanel::makeAnnotationMarker(const route_explore_msgs::IntersectionPtr intersection)
  {
    InteractiveMarker intersection_marker;
    intersection_marker.header.frame_id = "map";
    intersection_marker.pose.position = intersection->position;
    intersection_marker.pose.position.z = 0.1;
    intersection_marker.scale = 1;
    intersection_marker.name = intersection->name;

    intersectionMap_.insert({intersection->name, intersection});

    makePointControl(intersection_marker);
    if (selected_ == intersection->name) {
      setControl(intersection_marker, true, true, false);
    }
    server->insert(intersection_marker);
    server->setCallback(intersection_marker.name, boost::bind(&RouteAnnotationPanel::processFeedback, this, _1));
    menu_handler.apply(*server, intersection_marker.name);

    int count = 0;
    for (auto it = intersection->ways.begin(); it < intersection->ways.end(); it++) {
      InteractiveMarker way_marker;
      auto way = *it;
      way_marker.header.frame_id = "map";
      way_marker.pose = way.pose;
      way_marker.pose.position.z = 0.1;
      way_marker.scale = 1;
      way_marker.name = way.name;
      intersectionMap_.insert({way.name, intersection});
      count++;
      
      makeArrowControl(way_marker);
      if (selected_ == way.name) {
	setControl(way_marker, true, true, true);
      }
      server->insert(way_marker);
      server->setCallback(way_marker.name, boost::bind(&RouteAnnotationPanel::processFeedback, this, _1));
    }
  }
  
  void RouteAnnotationPanel::addAnnotationControl(tf::Vector3 &point)
  {
    auto now = ros::Time::now();
    std::ostringstream name;  
    name << "intersection_" << now.sec;

    auto intersection = boost::make_shared<route_explore_msgs::Intersection>();
    intersection->name = name.str();
    tf::pointTFToMsg(point, intersection->position);

    intersections_.push_back(intersection);
    updateMarkers();
  }

  void RouteAnnotationPanel::updateMarkers()
  {
    server->clear();

    QStringList list;
    for (auto it = intersections_.begin(); it < intersections_.end(); it++) {
      makeAnnotationMarker(*it);
      list << QString((*it)->name.c_str());
    }
    server->applyChanges();

    annotation_model_->setStringList(list);
  }

  void RouteAnnotationPanel::save(rviz::Config config) const
  {
    Panel::save(config);
  }

  void RouteAnnotationPanel::load(const rviz::Config & config)
  {
    Panel::load(config);
  }

} // end namespace mf_localization_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::RouteAnnotationPanel, rviz::Panel )
