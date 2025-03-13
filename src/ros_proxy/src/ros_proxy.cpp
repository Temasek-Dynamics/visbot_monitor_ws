/**
 * @file ros_proxy.cpp
 * @author Tiger Chen (tiger@visbot.com.cn)
 * @brief set up ros_proxy between ROS1 messages cross ethernet machines
 * including ROS message serialization/deserialization, message passing thru nanomsg.
 * @Usage 
 * 1. add one MESSAGE_TYPE, new ROS msg_, new pub_/sub_
 * 2. write local ROS topic callback.
 * 3. fill in handleing in nanomsg receiving thread
 *
 * @version 0.1
 * @date 2023-05-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <assert.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <controller_msgs/MinTraj.h>
#include <controller_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <controller_msgs/control.h>
#include <controller_msgs/SwarmMatrix.h>

#include <eigen3/Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <nanomsg/nn.h>
#include <nanomsg/bus.h>

// Peter Modification
#include <std_msgs/Bool.h>
#include <controller_msgs/TargetCompletion.h>
// Peter Modification End

using namespace std;

int drone_num_; //1-99
int drone_id_; //0 - drone_num_-1
std::vector<string> ip_list_;
bool is_commander_ = false;
bool send_pc_ = false;
string commander_ip_;
//string port = "18011";
int sock;
double odom_freq_ = 30.0;

//message type
enum MESSAGE_TYPE
{
  ODOM = 100,
  ONE_TRAJ,
  POINT_CLOUD,
  GOAL,
  INIT_POSE,
  CONTROL,
  SWARM_MATRIX,
  // Peter Modification
  TARGET_REACH,
  // Peter Modification End
} massage_type_;

//ROS messages
nav_msgs::Odometry odom_msg_;
controller_msgs::MinTraj MinTraj_msg_;
sensor_msgs::PointCloud2 pc_msg_;
//geometry_msgs::PoseStamped goal_msg_;
controller_msgs::PoseStamped   goal_msg_;
controller_msgs::PoseStamped   pose_msg_;
controller_msgs::control control_msg_;
controller_msgs::SwarmMatrix swarm_matrix_msg_;
// Peter Modification
// std_msgs::Bool target_reach_msg_;
controller_msgs::TargetCompletion target_reach_msg_;
// Peter Modification End

ros::Subscriber my_odom_sub_, my_traj_sub_, goal_sub_, control_sub_, my_pc_sub_, swarm_matrix_sub_;
ros::Publisher other_odoms_pub_, one_traj_pub_, goal_pub_, control_pub_, swarm_matrix_pub_;
std::vector<ros::Publisher> odom_pub_; //commander show all drone's odom
std::vector<ros::Publisher> traj_pub_; //commander show all drone's trajectory
std::vector<ros::Publisher> pc_pub_; //commander show all drone's point cloud
std::vector<ros::Publisher> target_reach_pub_; // Peter Modification: commander show all drone's target reach status
ros::Subscriber initpose_sub_; //commander all drone's init pose 
ros::Publisher initpose_pub_; //pub initpose for drone
ros::Subscriber target_reach_sub_; // Peter Modification: pub drone's target reach status via WiFi

void initSock() 
{
    string sockaddr;
    sock = nn_socket(AF_SP, NN_BUS);
    assert(sock >= 0);

    if(is_commander_) {
        sockaddr = commander_ip_;
        assert(nn_bind(sock, sockaddr.c_str()) >= 0);
        printf("[ros_proxy] bind socket:%s\n", sockaddr.c_str());
        sleep(1);

        for(int i = 0; i <drone_num_ ; i++) {
            sockaddr = ip_list_[i];
            assert(nn_connect(sock, sockaddr.c_str()) >= 0);
            printf("[ros_proxy] connect socket:%s\n", sockaddr.c_str());
        }
    } else {
        sockaddr = ip_list_[drone_id_];
        assert(nn_bind(sock, sockaddr.c_str()) >= 0);
        printf("[ros_proxy] bind socket:%s\n", sockaddr.c_str());
        sleep(1);

        for(int i = drone_id_+1; i <drone_num_ ; i++) {
            sockaddr = ip_list_[i];
            assert(nn_connect(sock, sockaddr.c_str()) >= 0);
            printf("[ros_proxy] connect socket:%s\n", sockaddr.c_str());
        }
    } 
    sleep(1);
}

//format: MESSAGE_TYPE, from, msg_size, msg
template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg, char * &buf)
{
  namespace ser = ros::serialization;
  uint32_t ros_msg_size = ser::serializationLength(msg);
  uint32_t msg_size= ros_msg_size + sizeof(MESSAGE_TYPE) + 2*sizeof(uint32_t) ;
  
  buf = (char*) malloc(sizeof(char) * msg_size);
  uint8_t * ptr = (uint8_t*)buf;

  //MESSAGE_TYPE
  *((MESSAGE_TYPE*)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  //from = drone_id_
  *((uint32_t*)ptr) = (uint32_t) drone_id_;
  ptr += sizeof(uint32_t);

  //msg_size
  *((uint32_t *)ptr) = ros_msg_size;
  ptr += sizeof(uint32_t);

  //msg
  ser::OStream stream(ptr, ros_msg_size);
  ser::serialize(stream, msg);

  return msg_size;
}

MESSAGE_TYPE getMsgType(uint8_t * buf)
{
  auto ptr = (uint8_t *)buf;
  MESSAGE_TYPE type = * ( (MESSAGE_TYPE*) ptr);
  return type;
}

int getFrom(uint8_t * buf)
{
  auto ptr = (uint8_t *)(buf+sizeof(MESSAGE_TYPE)); //skip MESSAGE_TYPE
  uint32_t id = * ( (uint32_t*) ptr);
  return id;
}

//format: MESSAGE_TYPE, from, to, msg_size, msg
template <typename T>
int deserializeTopic(uint8_t * buf, T &msg)
{
  //skip MESSAGE_TYPE+from
  auto ptr = (uint8_t *)(buf + sizeof(MESSAGE_TYPE) + sizeof(uint32_t)); 

  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  namespace ser = ros::serialization;
  ser::IStream stream(ptr, msg_size);
  ser::deserialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + 2*sizeof(uint32_t);
}

void point_cloud_callback(const sensor_msgs::PointCloud2Ptr &msg)
{
  char * buf = NULL;
  sensor_msgs::PointCloud2 pc_msg = *msg;
  pc_msg.header.frame_id = string("drone_") + std::to_string(drone_id_)+string("_world");

  int len = serializeTopic(MESSAGE_TYPE::POINT_CLOUD, pc_msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
  delete(buf);
}

void odom_callback(const nav_msgs::OdometryPtr &msg)
{
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  //msg->child_frame_id = string("drone_") + std::to_string(drone_id_);
  msg->header.frame_id = string("drone_") + std::to_string(drone_id_)+string("_world");

  char * buf;
  int len = serializeTopic(MESSAGE_TYPE::ODOM, *msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
  delete(buf);
}

void traj_callback(const controller_msgs::MinTrajPtr &msg)
{
  char * buf;
  int len = serializeTopic(MESSAGE_TYPE::ONE_TRAJ, *msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
  delete(buf);
}

void goal_callback(const controller_msgs::PoseStampedPtr &msg)
{
    // Peter Modification
    // Output the received message details using ROS logging system
    ROS_INFO("Received PoseStamped Message:");
    ROS_INFO("Header:");
    ROS_INFO("  Seq: %d", msg->header.seq);
    ROS_INFO("  Stamp: %f", msg->header.stamp.toSec());
    ROS_INFO("  Frame_id: %s", msg->header.frame_id.c_str());
    ROS_INFO("Pose:");
    ROS_INFO("  Position:");
    ROS_INFO("    x: %f", msg->pose.position.x);
    ROS_INFO("    y: %f", msg->pose.position.y);
    ROS_INFO("    z: %f", msg->pose.position.z);
    ROS_INFO("  Orientation:");
    ROS_INFO("    w: %f", msg->pose.orientation.w);
    ROS_INFO("    x: %f", msg->pose.orientation.x);
    ROS_INFO("    y: %f", msg->pose.orientation.y);
    ROS_INFO("    z: %f", msg->pose.orientation.z);
    // Peter Modification End
  char * buf = NULL;
  int len = serializeTopic(MESSAGE_TYPE::GOAL, *msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
    // Peter Modification
    if (send == len) {
        ROS_INFO("Successfully sent PoseStamped message.");
    } else {
        ROS_ERROR("Failed to send the entire message, sent %d bytes of %d.", send, len);
    }
    // Peter Modification End
  delete(buf);
}

void initpose_callback(const controller_msgs::PoseStampedPtr &msg)
{
  char * buf = NULL;
  int len = serializeTopic(MESSAGE_TYPE::INIT_POSE, *msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
  delete(buf);
}

void control_callback(const controller_msgs::controlPtr &msg)
{
  char * buf = NULL;
  int len = serializeTopic(MESSAGE_TYPE::CONTROL, *msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
    // Peter Modification
    if (send == len) {
        ROS_INFO("Successfully sent Control message.");
    } else {
        ROS_ERROR("Failed to send the entire Control message, sent %d bytes of %d.", send, len);
    }
    // Peter Modification End
  delete(buf);
}

void swarm_matrix_callback(const controller_msgs::SwarmMatrixPtr &msg)
{
  char * buf = NULL;
  int len = serializeTopic(MESSAGE_TYPE::SWARM_MATRIX, *msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
}

// Peter Modification
void target_reach_callback(const std_msgs::BoolPtr &msg)
{
  controller_msgs::TargetCompletion target_msg;
  target_msg.header.stamp = ros::Time::now();
  target_msg.header.frame_id = "world";
  target_msg.drone_id = drone_id_;
  target_msg.is_reached = msg->data;
  // Serialize TargetCompletion message
  char * buf = NULL;
  int len = serializeTopic(MESSAGE_TYPE::TARGET_REACH, target_msg, buf);
  int send = nn_send(sock, buf, len, 0);
  assert(send == len);
  delete(buf);
  ROS_INFO("Successfully sent TargetCompletion message.");
}
// Peter Modification End

void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id, bool show_sphere = true) 
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
}

void visualizeTrajectory(const controller_msgs::MinTraj &msg, int id)
{
  int index = msg.drone_id;
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d point;
  point[0] = msg.start_p[0]; point[1] = msg.start_p[1]; point[2] = msg.start_p[2];
  points.push_back(point);
  int size = msg.duration.size();
  for(int i = 0 ; i < size - 1 ; i++) {
     point[0] = msg.inner_x[i];
     point[1] = msg.inner_y[i];
     point[2] = msg.inner_z[i];
     points.push_back(point);
  }
  point[0] = msg.end_p[0]; point[1] = msg.end_p[1]; point[2] = msg.end_p[2];
  points.push_back(point);
  //std::cout << "TIGER DEBUG: trajlist[" << index << "] size: " << size+1 << std::endl;
  displayMarkerList(traj_pub_[index], points, 0.15, Eigen::Vector4d(1, 0, 0, 1), id);
}

void recv_fun()
{
  //int to = 100;
  //assert(nn_setsockopt(sock, NN_SOL_SOCKET, NN_RCVTIMEO, &to, sizeof(to)) >= 0);

  while (true)
  {
    char * buf;
    int recv = nn_recv(sock, &buf, NN_MSG, 0);
    if(recv>0) {
        uint8_t *ptr = (uint8_t*)buf;
        MESSAGE_TYPE type = getMsgType(ptr);
        //printf("[ros_proxy] RECEIVED[%d]: type: '%d' \n", recv, type);
        switch (type) {
            case MESSAGE_TYPE::ODOM:
            {
                if (recv == deserializeTopic(ptr, odom_msg_))
                {
                    if(is_commander_) {
                        int index;
                        sscanf(odom_msg_.header.frame_id.c_str(), "drone_%d_world", &index);
                        //std::cout << "TIGER: odom droneID:" << index << std::endl;
                        if(index >=-1 && index <drone_num_) {
                            odom_msg_.header.frame_id = "world";
                            odom_pub_[index].publish(odom_msg_);
                        }
                    } else {
                        other_odoms_pub_.publish(odom_msg_);
                    }
                }
                else
                {
                    ROS_ERROR("[ros_proxy] Received message length not matches the sent one (ODOM)!!!");
                    continue;
                }
                break;
            }
            case MESSAGE_TYPE::ONE_TRAJ:
            {
                if (recv == deserializeTopic(ptr, MinTraj_msg_))
                {
                    if(is_commander_) {
                        visualizeTrajectory(MinTraj_msg_, 0);
                    } else {
                        one_traj_pub_.publish(MinTraj_msg_);
                    }
                }
                else
                {
                    ROS_ERROR("[ros_proxy] Received message length not matches the sent one (ONE_TRAJ)!!!");
                    continue;
                }
                break;
            }
            case MESSAGE_TYPE::POINT_CLOUD:
            {
                if (recv == deserializeTopic(ptr, pc_msg_))
                {
                    if(is_commander_) {
                        int index;
                        sscanf(pc_msg_.header.frame_id.c_str(), "drone_%d_world", &index);
                        if(index >=-1 && index <=drone_num_) {
                            pc_msg_.header.frame_id = "world";
                            pc_pub_[index].publish(pc_msg_);
                        }
                    }
                }
                break;
            }
            case MESSAGE_TYPE::GOAL:
            {
                if (recv == deserializeTopic(ptr, goal_msg_))
                {
                    if((goal_msg_.drone_id == drone_id_ || goal_msg_.drone_id == -1) && !is_commander_) {
                        geometry_msgs::PoseStamped msg;
                        msg.header = goal_msg_.header;
                        msg.pose = goal_msg_.pose;
                        goal_pub_.publish(msg);
                        //goal_pub_.publish(goal_msg_);
                    }
                }
                else
                {
                    ROS_ERROR("[ros_proxy] Received message length not matches the sent one (GOAL)!!!");
                    continue;
                }
                break;
            }
            case MESSAGE_TYPE::INIT_POSE:
            {
                if (recv == deserializeTopic(ptr, pose_msg_))
                {
                    if(pose_msg_.drone_id == drone_id_ && !is_commander_) {
                        geometry_msgs::PoseStamped msg;
                        msg.header = pose_msg_.header;
                        msg.pose = pose_msg_.pose;
                        initpose_pub_.publish(msg);
                    }
                }
                else
                {
                    ROS_ERROR("[ros_proxy] Received message length not matches the sent one (INIT_POSE)!!!");
                    continue;
                }
                break;
            }
            case MESSAGE_TYPE::CONTROL:
            {
                if(!is_commander_) { //don't send control to commander
                    if (recv == deserializeTopic(ptr, control_msg_))
                    {
                        control_pub_.publish(control_msg_);
                    }
                    else
                    {
                        ROS_ERROR("[ros_proxy] Received message length not matches the sent one (CONTROL)!!!");
                        continue;
                    }
                }
                break;
            }
            case MESSAGE_TYPE::SWARM_MATRIX:
            {
                if(!is_commander_) { //don't send control to commander
                    if (recv == deserializeTopic(ptr, swarm_matrix_msg_))
                    {
                        swarm_matrix_msg_.drone_num = drone_id_;
                        swarm_matrix_pub_.publish(swarm_matrix_msg_);
                    }
                    else
                    {
                        ROS_ERROR("[ros_proxy] Received message length not matches the sent one (SWARM_MATRIX)!!!");
                        continue;
                    }
                }
                break;
            }
            // Peter Modification
            case MESSAGE_TYPE::TARGET_REACH:
            {
                if(is_commander_) { // Only publish the message if the node is a commander
                    if (recv == deserializeTopic(ptr, target_reach_msg_))
                    {
                        int index;
                        index = target_reach_msg_.drone_id;
                        std_msgs::Bool msg;
                        msg.data = target_reach_msg_.is_reached;
                        target_reach_pub_[index].publish(msg);
                        
                    }
                }
                break;
            }
            // Peter Modification End
            default:
                ROS_ERROR("[ros_proxy] Unknown received message type???");
                break;
        }
        nn_freemsg(buf);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_proxy");
  ros::NodeHandle nh("~");

  nh.param("odom_hz", odom_freq_, 100.0);
  nh.param("drone_id", drone_id_, -1);
  nh.param("drone_num", drone_num_, 0);
  nh.param("is_commander", is_commander_, false);
  nh.param("send_point_cloud", send_pc_, false);
  nh.param("commander_ip", commander_ip_, string("127.0.0.1"));
  ip_list_.resize(drone_num_);
  for (int i = 0; i < drone_num_ ; ++i)
  {
    nh.param("drone_ip_" + to_string(i), ip_list_[i], string("127.0.0.1"));
  }

  if (drone_id_ == -1 || drone_id_ > drone_num_)
  {
    ROS_WARN("[ros_proxy] Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  initSock();

  my_odom_sub_ = nh.subscribe("my_odom", 10, odom_callback, ros::TransportHints().tcpNoDelay());
  other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);

  my_traj_sub_ = nh.subscribe("/broadcast_traj_from_planner", 100, traj_callback, ros::TransportHints().tcpNoDelay());
  one_traj_pub_ = nh.advertise<controller_msgs::MinTraj>("/broadcast_traj_to_planner", 100);

  my_pc_sub_ = nh.subscribe("point_cloud", 100, point_cloud_callback, ros::TransportHints().tcpNoDelay());
  
  if(is_commander_) {
    initpose_sub_ = nh.subscribe("/initpose", 100, initpose_callback, ros::TransportHints().tcpNoDelay());
    control_sub_ = nh.subscribe("/control", 100, control_callback, ros::TransportHints().tcpNoDelay());
    swarm_matrix_sub_ = nh.subscribe("/swarm_matrix", 10, swarm_matrix_callback, ros::TransportHints().tcpNoDelay());
    odom_pub_.resize(drone_num_);
    traj_pub_.resize(drone_num_);
    pc_pub_.resize(drone_num_);
    // Peter Modification
    target_reach_pub_.resize(drone_num_);
    // Peter Modification End
    for(int i =0 ; i<drone_num_; i++) {
        goal_sub_ = nh.subscribe("/goal", 100, goal_callback, ros::TransportHints().tcpNoDelay());
        string odom_name = string("/drone_") + std::to_string(i)+string("_mavros/local_position/odom");
        odom_pub_[i] = nh.advertise<nav_msgs::Odometry>(odom_name, 100);
        string traj_name = string("/drone_") + std::to_string(i)+string("_planner/trajectory");
        traj_pub_[i] = nh.advertise<visualization_msgs::Marker>(traj_name, 100);
        string pc_name = string("/drone_") + std::to_string(i)+string("_planner/point_cloud");
        pc_pub_[i] = nh.advertise<sensor_msgs::PointCloud2>(pc_name, 100);
        // Peter Modification
        string target_reach_name = string("/drone_") + std::to_string(i)+string("_planner/target_reach");
        target_reach_pub_[i] = nh.advertise<std_msgs::Bool>(target_reach_name, 100);
        // Peter MOdification End
    }
  } else {
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
    initpose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/initpose", 100);
    control_pub_ = nh.advertise<controller_msgs::control>("/control", 100);
    swarm_matrix_pub_ = nh.advertise<controller_msgs::SwarmMatrix>("/swarm_matrix", 10);
    // Peter Modification
    target_reach_sub_ = nh.subscribe("/target_reach", 100, target_reach_callback, ros::TransportHints().tcpNoDelay());
    // Peter Modification End
  }

  boost::thread recv_thd(recv_fun);
  recv_thd.detach();
  ros::Duration(0.1).sleep();

  ros::spin();

  return 0;
}
