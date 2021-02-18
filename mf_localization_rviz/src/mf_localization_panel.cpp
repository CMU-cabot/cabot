// Copyright (c) 2021  IBM Corporation
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

#include "mf_localization_panel.h"
#include "mf_localization_msgs/RestartLocalization.h"
#include "mf_localization_msgs/FloorChange.h"
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace rviz
{

  MultifloorLocalizationPanel::MultifloorLocalizationPanel( QWidget* parent )
    : Panel( parent )
  {
    // init components
    restart_button_ = new QPushButton( "Restart Localization" );
    up_button_ = new QPushButton( "Up" );
    down_button_ = new QPushButton( "Down" );
    memo_button_ = new QPushButton( "Memo" );
  
    auto label = new QLabel("Floor:");
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // layout
    QVBoxLayout* vlayout = new QVBoxLayout;
    QHBoxLayout* hlayout1 = new QHBoxLayout;
    QHBoxLayout* hlayout2 = new QHBoxLayout;
    QHBoxLayout* hlayout3 = new QHBoxLayout;
  
    hlayout1->addWidget(restart_button_);
  
    hlayout2->addWidget(label);
    hlayout2->addWidget(up_button_);
    hlayout2->addWidget(down_button_);

    hlayout3->addWidget(memo_button_);


    auto lineA = new QFrame;
    lineA->setFrameShape(QFrame::HLine);
    lineA->setFrameShadow(QFrame::Sunken);
    auto lineB = new QFrame;
    lineB->setFrameShape(QFrame::HLine);
    lineB->setFrameShadow(QFrame::Sunken);

    vlayout->addLayout(hlayout1);
    vlayout->addWidget(lineA);
    vlayout->addLayout(hlayout2);
    vlayout->addWidget(lineB);
    vlayout->addLayout(hlayout3);
    
    setLayout( vlayout );

    QObject::connect( restart_button_, SIGNAL(clicked()), this, SLOT(sendRestartLocalization()));
    QObject::connect( up_button_, SIGNAL(clicked()), this, SLOT(sendFloorUp()));
    QObject::connect( down_button_, SIGNAL(clicked()), this, SLOT(sendFloorDown()));
    QObject::connect( memo_button_, SIGNAL(clicked()), this, SLOT(sendPromptMemo()));
  }

  MultifloorLocalizationPanel::~MultifloorLocalizationPanel()
  {
  
  }
  
  void MultifloorLocalizationPanel::onInitialize()
  {
    updateTopic();
    updateService();
  }

  void MultifloorLocalizationPanel::updateTopic()
  {
    try
    {
      pub_ = nh_.advertise<std_msgs::String>("memo", 1);
    }
    catch (const ros::Exception& e)
    {
      ROS_ERROR_STREAM_NAMED("GoalTool", e.what());
    }
  }
  
  void MultifloorLocalizationPanel::updateService()
  {
    try {
    
      restart_localization_client_ =
	nh_.serviceClient<mf_localization_msgs::RestartLocalization>("restart_localization");
    
      floor_change_client_ =
	nh_.serviceClient<mf_localization_msgs::FloorChange>("floor_change");
    
    } catch (const ros::Exception& e) {
      ROS_ERROR_STREAM_NAMED("MultifloorLocalizationPanel", e.what());
    }  
  }

  void MultifloorLocalizationPanel::sendRestartLocalization() 
  {
    mf_localization_msgs::RestartLocalization srv;
    restart_localization_client_.call(srv);
  }

  void MultifloorLocalizationPanel::sendFloorUp()
  {
    sendFloorChange(+1);
  }
  
  void MultifloorLocalizationPanel::sendFloorDown()
  {
    sendFloorChange(-1);
  }

  void MultifloorLocalizationPanel::sendPromptMemo()
  {
    bool ok;
    QInputDialog *dialog = new QInputDialog(this);
    dialog->setLabelText(tr("Input Debug Memo"));
    dialog->setInputMode(QInputDialog::InputMode::TextInput);
    dialog->resize(500, 100);
    ok = dialog->exec();
    QString text = dialog->textValue();
    
    //QString text = QInputDialog::getText(this, tr("Input Debug Memo"),
    //tr("memo:"), 
    //tr(""), &ok);
    
    if (ok && !text.isEmpty()) {
      std_msgs::String msg;
      msg.data = text.toStdString();

      pub_.publish(msg);
      ROS_INFO("DebugMemo: %s", text.toStdString().c_str());
    }
  }
  
  
  void MultifloorLocalizationPanel::sendFloorChange(int diff)
  {
    mf_localization_msgs::FloorChange srv;
    srv.request.diff.data = diff;
    floor_change_client_.call(srv);
  }
  
  void MultifloorLocalizationPanel::save(rviz::Config config) const
  {
    Panel::save(config);
  }

  void MultifloorLocalizationPanel::load(const rviz::Config & config)
  {
    Panel::load(config);
  }


} // end namespace mf_localization_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::MultifloorLocalizationPanel, rviz::Panel )

