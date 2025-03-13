#ifndef SWARM_TOOL_H
#define SWARM_TOOL_H

#include <QObject>
#include <ros/ros.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include "rviz/default_plugin/tools/pose_tool.h"
#include "rviz/default_plugin/tools/interaction_tool.h"
// Peter Modification
#include <std_msgs/Bool.h>
#include <rviz/tool.h>
// Peter Modification End

using namespace rviz;

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;
class FloatProperty;

class SwarmControl {
public:
   SwarmControl() {}
   SwarmControl(int cmd);
   ~SwarmControl() {}
   void control();
private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_;
    int cmd_;
};

class Start : public rviz::InteractionTool
{
public:
    Start() : controller(0) {}
    ~Start() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class Stop : public rviz::InteractionTool
{
public:
    Stop() : controller(1) {}
    ~Stop() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class Arm : public rviz::InteractionTool
{
public:
    Arm() : controller(10) {}
    ~Arm() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class Unarm : public rviz::InteractionTool
{
public:
    Unarm() : controller(11) {}
    ~Unarm() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class Takeoff : public rviz::InteractionTool
{
public:
    Takeoff() : controller(20) {}
    ~Takeoff() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class Land : public rviz::InteractionTool
{
public:
    Land() : controller(21) {}
    ~Land() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class Offboard : public rviz::InteractionTool
{
public:
    Offboard() : controller(15) {}
    ~Offboard() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

class PosCtl : public rviz::InteractionTool
{
public:
    PosCtl() : controller(16) {}
    ~PosCtl() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

// Peter Modification
class SetWayPoint : public rviz::InteractionTool
{
public:
    SetWayPoint() : controller(102) {}
    ~SetWayPoint() override {}
    void onInitialize() override;
    void activate() override;

private:
    SwarmControl controller;
};

// class SetSeqPoint : public rviz::InteractionTool
// {
//     Q_OBJECT
// public:
//     SetSeqPoint();
//     ~SetSeqPoint() override;
//     void onInitialize() override;
//     void activate() override;

// private Q_SLOTS:
//   void onButtonClick();

// private:
//   ros::NodeHandle nh_;
//   ros::Publisher state_pub_;
// };
// Peter Modification End

class SetInitPose : public rviz::PoseTool
{
    Q_OBJECT
public:
    SetInitPose();
    ~SetInitPose() override
    {
    }
    void onInitialize() override;
    void activate() override;
protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher initpose_pub_;

  //ros::NodeHandle nh2_;
  //ros::Publisher pub2_;

  rviz::IntProperty* droneid_property_;
  rviz::FloatProperty* x_;
  rviz::FloatProperty* y_;
  rviz::FloatProperty* z_;
  rviz::FloatProperty* yaw_;
};


} // namespace rviz

#endif
