#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Accel,PoseArray,AccelStamped, Twist
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool,Float32
from controller_msgs.msg import control,SwarmMatrix
import math
import time
from tf.transformations import quaternion_from_euler
"""
    Task: go to current preset waypoint(swarm center), with Obstacle Avoidance

    Returns:
        _type_: _description_
"""
class SequentialWpsMission:
    def __init__(self):
        # single drone test: manually set /target_reach and switch to next waypoint
        self.is_manual_reach = rospy.get_param("/is_manual", True)
        
        # Parameters and waypoints values
        self.drone_num = rospy.get_param("/drone_num",1)
        self.waypoint_num=rospy.get_param("/waypoint_num",1)
        self.wps_list=[[None,None,None,None] for _ in range(self.waypoint_num)]

        for i in range(self.waypoint_num):
            self.wps_list[i][0]=rospy.get_param("/waypoint"+str(i)+"_x",0)
            self.wps_list[i][1]=rospy.get_param("/waypoint"+str(i)+"_y",0)
            self.wps_list[i][2]=rospy.get_param("/waypoint"+str(i)+"_z",1.5)
            self.wps_list[i][3]=rospy.get_param("/waypoint"+str(i)+"_yaw_rad",0)
        
        self.wps_msg_list = []
        self.current_wp = 0
        
        # Swarm Matrix
        swarm_yaw = rospy.get_param('/swarm_yaw', 1.571)
        swarm_positions = [[None, None, None, None] for _ in range(self.drone_num)]
        for i in range(self.drone_num):
            swarm_positions[i][0] = rospy.get_param("/swarm{}_x".format(i), 0.0)
            swarm_positions[i][1] = rospy.get_param("/swarm{}_y".format(i), 0.0)
            swarm_positions[i][2] = rospy.get_param("/swarm{}_z".format(i), 0.0)
            swarm_positions[i][3] = rospy.get_param("/swarm{}_yaw".format(i), 0.0)
            
        self.swarm_matrix = SwarmMatrix()
        self.swarm_matrix.header.seq = 1
        self.swarm_matrix.header.stamp = rospy.Time.now()
        self.swarm_matrix.header.frame_id = 'world'
        self.swarm_matrix.drone_num = self.drone_num
        self.swarm_matrix.swarm_yaw = swarm_yaw
        self.swarm_matrix.x = [pos[0] for pos in swarm_positions]
        self.swarm_matrix.y = [pos[1] for pos in swarm_positions]
        self.swarm_matrix.z = [pos[2] for pos in swarm_positions]
        self.swarm_matrix.yaw = [pos[3] for pos in swarm_positions]
     
        ## Subscribers and Publishers
        self.drone_target_reach_sub_list=[]
        
        if self.is_manual_reach:
            self.manual_switch = rospy.Subscriber("/manual_switch", Bool, self.manual_switch_callback)
        else:
            for i in range(self.drone_num):
                topic_name=f"/drone_{i}_target_reach"
                self.drone_target_reach_sub_list.append(rospy.Subscriber(topic_name, Bool, self.drone_target_reach_callback,callback_args=i))
        
        self.wps_pose_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.swarm_matrix_pub = rospy.Publisher("/swarm_matrix", SwarmMatrix, queue_size=10)
        self.captain_task_pub = rospy.Publisher("/control", control, queue_size=10)

        ## drone states
        self.manual_switch = True
        self.ALL_REACH = True
        self.drone_reach_list = [self.ALL_REACH for i in range(self.drone_num)]


    def create_pose(self, x, y, z,yaw_rad):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp=rospy.Time.now()
        pose_stamped.header.frame_id="world"

        pose_stamped.pose.position.x = float(x) 
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z) 
        
        quat = quaternion_from_euler(0, 0, yaw_rad)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        return pose_stamped
    


    def load_wps_msg_list(self):
        # print(len(self.wps_list))
        for i in range(len(self.wps_list)):
            self.wps_msg_list.append(self.create_pose(self.wps_list[i][0], 
                                                      self.wps_list[i][1], 
                                                      self.wps_list[i][2],
                                                      self.wps_list[i][3]))
    
    def drone_target_reach_callback(self, msg,drone_id):
        self.drone_reach_list[drone_id] = msg.data
        
    def manual_switch_callback(self, msg):
        self.manual_switch = msg.data

    def pub_latest_wps(self):
        
        self.ALL_REACH = all(self.drone_reach_list)
        if self.current_wp < len(self.wps_msg_list):
            
            if self.is_manual_reach==False:
                NEXT_COND=self.ALL_REACH
                print("drone_reach_list: ", self.drone_reach_list)
            else:
                NEXT_COND=self.manual_switch
                print("manual_switch is: ", self.manual_switch)

            if NEXT_COND:

                # publish the next waypoint
                print("will move to: \n",self.wps_msg_list[self.current_wp])
                self.wps_pose_pub.publish(self.wps_msg_list[self.current_wp])


                self.pub_captain_task()
                print("has published the captain task")
                
                # Update the current waypoint 
                # and drone_reach_list
                self.current_wp += 1
                
                self.ALL_REACH = False
                self.manual_switch = False

                self.drone_reach_list = [self.ALL_REACH for i in range(self.drone_num)]
            else:
                print("wait for next switch")
        else:
            print("All waypoints sent")

    def pub_swarm_matrix(self):
        self.swarm_matrix_pub.publish(self.swarm_matrix)
        print(self.swarm_matrix)
        print("has published the swarm matrix\n")
    
    def pub_captain_task(self):
        captain_task_cmd = control()
        captain_task_cmd.header.stamp = rospy.Time.now()
        captain_task_cmd.cmd=102 # 102: go to current preset waypoint(swarm center), with Obstacle Avoidance
        captain_task_cmd.drone_id=-1
        self.captain_task_pub.publish(captain_task_cmd)

def str_to_bool(s):
    if s.lower() in ['yes']:
        return True
    elif s.lower() in ['no']:
        return False
    else:
        return False
    
if __name__ == '__main__':

    rospy.init_node('seq_wps_mission')
    seq_wps_m = SequentialWpsMission()
    seq_wps_m.load_wps_msg_list()
    rospy.sleep(1)
    # Publish the swarm matrix after rospy.sleep(1) or the topic /swarm_matrix will not get the message
    seq_wps_m.pub_swarm_matrix()
    print("#"*50)
    print("-"*50)
    print("#"*50)    
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if seq_wps_m.current_wp < len(seq_wps_m.wps_msg_list):
            print("next waypoint is: ", seq_wps_m.wps_msg_list[seq_wps_m.current_wp], "\n")
          
            manual_switch_input = input("Enter 'yes' to switch to next waypoint : \n ")
            seq_wps_m.manual_switch = str_to_bool(manual_switch_input)
            
            print("-"*50)
            print("this round input result is:\n")

            if seq_wps_m.manual_switch:
                seq_wps_m.pub_latest_wps() 
                print("has published the latest waypoint and execute now")    

            else:
                print("wait for next switch, nothing published")

            print("#"*50)
            print("-"*50)
            print("#"*50)

            rate.sleep()
        else:
            print("All waypoints sent")
            break