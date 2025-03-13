#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <geometry_msgs/Pose2D.h>

#include "swarm_tool.h"

#include "controller_msgs/control.h"
#include "controller_msgs/PoseStamped.h"

// Peter Modification
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/visualization_manager.h>
#include <rviz/panel_dock_widget.h>
// Peter Modification End

using namespace  rviz;

namespace rviz
{

SwarmControl::SwarmControl(int cmd) 
{
  cmd_ = cmd;
  try
  {
    control_pub_ = nh_.advertise<controller_msgs::control>("/control", 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("control", e.what());
  }
}

void SwarmControl::control() {
  controller_msgs::control controlMsg;
  controlMsg.header.stamp = ros::Time::now();
  controlMsg.cmd = cmd_;
  controlMsg.drone_id = -1;
  ROS_INFO("publish control msg: %d", cmd_);
  control_pub_.publish(controlMsg);
}

// // Peter Modification
// SetSeqPoint::SetSeqPoint()
// { 
//   try
//   {
//     state_pub_ = nh_.advertise<std_msgs::Bool>("/mission_state", 1);
//   }
//    catch (const ros::Exception& e)
//   {
//     ROS_ERROR_STREAM_NAMED("TargetCompletion state", e.what());
//   }
// }
// // Peter Modification End

SetInitPose::SetInitPose()
{
  //shortcut_key_ = 'i'; 

  //topic_property_ =
  droneid_property_ =
      new IntProperty("Drone_id", -1, "Drone_id to which to publish initial pose estimates.",
                         getPropertyContainer(), SLOT(updateTopic()), this);
  x_ = new FloatProperty("X", 0.5, "X for initial pose [m]", getPropertyContainer());
  y_ = new FloatProperty("Y", 0.5, "Y for initial pose [m]", getPropertyContainer());
  z_ = new FloatProperty("Z", 1.0, "Z for initial pose [m]", getPropertyContainer());
  yaw_ = new FloatProperty("Yaw", 0.0, "Yaw for initial pose [rad]", getPropertyContainer());

  /*
  try
  {
    pub2_ = nh2_.advertise<geometry_msgs::Pose2D>("/control", 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("control", e.what());
  }
  */

}

void SetInitPose::onInitialize()
{
  PoseTool::onInitialize();
  setName("SetInitPose");
  updateTopic();
}

void SetInitPose::updateTopic()
{
  try
  {
    std::string topic_name;
    initpose_pub_ = nh_.advertise<controller_msgs::PoseStamped>("/initpose", 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("SetInitPose", e.what());
  }
}

void SetInitPose::activate() {
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  controller_msgs::PoseStamped pose;
  pose.drone_id = droneid_property_->getInt();
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = x_->getFloat();
  pose.pose.position.y = y_->getFloat();
  pose.pose.position.z = z_->getFloat();

  geometry_msgs::Quaternion quat_msg;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw_->getFloat());
  pose.pose.orientation = tf2::toMsg(quat);
  ROS_INFO("Setting pose: %.3f %.3f %.3f, %.3f [frame=%s]", x_->getFloat(), y_->getFloat(), z_->getFloat(), yaw_->getFloat(), fixed_frame.c_str());
  initpose_pub_.publish(pose);
}

void SetInitPose::onPoseSet(double x, double y, double theta)
{
  //concel the poseset op.
  return;
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  controller_msgs::PoseStamped pose;
  pose.drone_id = droneid_property_->getInt();
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 1.0;


  geometry_msgs::Quaternion quat_msg;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.orientation = tf2::toMsg(quat);
  ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
  initpose_pub_.publish(pose);

  /*
  if (pub2_)
  {
    // unstamped Pose2D message
    geometry_msgs::Pose2D::Ptr pose_msg;
    pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
    pose_msg->x = x;
    pose_msg->y = y;
    pose_msg->theta = theta;
    pub2_.publish(pose_msg);
  }
  */
}


void Start::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Start");
}

void Start::activate() {
  //ROS_INFO("Start cmd send!");
  controller.control();
}

void Stop::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Stop");
}

void Stop::activate() {
  //ROS_INFO("Stop cmd send!");
  controller.control();
}

void Arm::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Arm");
}

void Arm::activate() {
  //ROS_INFO("Arm cmd send!");
  controller.control();
}

void Unarm::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Unarm");
}

void Unarm::activate() {
  //ROS_INFO("Unarm cmd send!");
  controller.control();
}

void Takeoff::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Takeoff");
}

void Takeoff::activate() {
  //ROS_INFO("Takeoff cmd send!");
  controller.control();
}

void Land::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Land");
}

void Land::activate() {
  //ROS_INFO("Land cmd send!");
  controller.control();
}

void Offboard::onInitialize()
{
  InteractionTool::onInitialize();
  setName("Offboard");
}

void Offboard::activate() {
  //ROS_INFO("Offboard cmd send!");
  controller.control();
}

void PosCtl::onInitialize()
{
  InteractionTool::onInitialize();
  setName("PosCtl");
}

void PosCtl::activate() {
  //ROS_INFO("PosCtl cmd send!");
  controller.control();
}

} // end namespace rviz

// Peter Modification
void SetWayPoint::onInitialize()
{
  InteractionTool::onInitialize();
  setName("SetWayPoint");
}

void SetWayPoint::activate() {
  //ROS_INFO("SetWayPoint cmd send!");
  controller.control();
}

// void SetSeqPoint::onInitialize()
// {
//   rviz::Tool::onInitialize();
//   setName("SetSeqPoint");
//   // Create a widget to hold the button
//   QWidget* tool_widget = new QWidget;
//   QVBoxLayout* layout = new QVBoxLayout;
//   // Create the button
//   QPushButton* button = new QPushButton("Publish Bool");
//   layout->addWidget(button);
//   tool_widget->setLayout(layout);
//   // Add the widget to the RViz tool panel
//   // context_->getToolManager()->getToolPanel()->layout()->addWidget(tool_widget);
//   rviz::PanelDockWidget* panel_dock_widget = new rviz::PanelDockWidget("SetSeqPoint", tool_widget);
//   context_->addPanelDockWidget(panel_dock_widget);
//   // Connect the button click signal to the slot
//   connect(button, &QPushButton::clicked, this, &SetSeqPoint::onButtonClick);
// }

// void SetSeqPoint::onButtonClick()
// {
//   //ROS_INFO("SetSeqPoint cmd send!");
//   // Create and publish the Bool message
//   std_msgs::Bool msg;
//   msg.data = true;  // Set the value as needed
//   state_pub_.publish(msg);
// }
// // Peter Modification End

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::SetInitPose, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Start, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Stop, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Arm, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Unarm, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Takeoff, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Land, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::Offboard, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(rviz::PosCtl, rviz::Tool)
// Peter Modification
PLUGINLIB_EXPORT_CLASS(rviz::SetWayPoint, rviz::Tool)
// PLUGINLIB_EXPORT_CLASS(rviz::SetSeqPoint, rviz::Tool)
// Peter Modification End