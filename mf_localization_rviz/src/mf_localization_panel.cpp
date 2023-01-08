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

#include <memory>
#include "mf_localization_rviz/mf_localization_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>

namespace rviz_common
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
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    updateTopic();
    updateService();
  }

  void MultifloorLocalizationPanel::updateTopic()
  {
    pub_ = node_->create_publisher<std_msgs::msg::String>("memo", 1);
  }
  
  void MultifloorLocalizationPanel::updateService()
  {
    restart_localization_client_ =
        node_->create_client<mf_localization_msgs::srv::RestartLocalization>("restart_localization");
    
    floor_change_client_ =
        node_->create_client<mf_localization_msgs::srv::FloorChange>("floor_change");
  }

  void MultifloorLocalizationPanel::sendRestartLocalization() 
  {
    RCLCPP_INFO(node_->get_logger(), "sendRestartLocalization");
    auto req = std::make_shared<mf_localization_msgs::srv::RestartLocalization::Request>();
    restart_localization_client_->async_send_request(req);
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
      std_msgs::msg::String msg;
      msg.data = text.toStdString();

      pub_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "DebugMemo: %s", text.toStdString().c_str());
    }
  }
  
  
  void MultifloorLocalizationPanel::sendFloorChange(int diff)
  {
    RCLCPP_INFO(node_->get_logger(), "sendFloorChange");
    auto req = std::make_shared<mf_localization_msgs::srv::FloorChange::Request>();
    req->diff.data = diff;
    floor_change_client_->async_send_request(req);
  }
  
  void MultifloorLocalizationPanel::save(rviz_common::Config config) const
  {
    Panel::save(config);
  }

  void MultifloorLocalizationPanel::load(const rviz_common::Config & config)
  {
    Panel::load(config);
  }


}  // end namespace rviz_common

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_common::MultifloorLocalizationPanel, rviz_common::Panel)
