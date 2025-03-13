#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import  Accel,PoseArray,AccelStamped, Twist
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool,Float32
from controller_msgs.msg import control,SwarmMatrix,PoseStamped
import math
import time
from tf.transformations import quaternion_from_euler
class InitPosePub:
    def __init__(self):
        self.init_pose_pub=rospy.Publisher("/initpose",PoseStamped,queue_size=10)

        # Parameters and waypoints values
        self.drone_num = rospy.get_param("/drone_num",1)
        self.init_pose_list = [[None, None, None, None, None] for _ in range(self.drone_num)]
        for i in range(self.drone_num):
            self.init_pose_list[i][0]=rospy.get_param("/drone_id_{}".format(i), 0)
            self.init_pose_list[i][1]=rospy.get_param("/init_pose_{}_x".format(i), 0)   
            self.init_pose_list[i][2]=rospy.get_param("/init_pose_{}_y".format(i), 0)
            self.init_pose_list[i][3]=rospy.get_param("/init_pose_{}_z".format(i), 0)
            self.init_pose_list[i][4]=rospy.get_param("/init_pose_{}_yaw_rad".format(i), 0)

    def publish_init_pose(self):
        for i in range(self.drone_num):
            init_pose_msg = PoseStamped()
            
            init_pose_msg.drone_id = self.init_pose_list[i][0]
            init_pose_msg.header.frame_id = 'world'
            init_pose_msg.header.stamp = rospy.Time.now()
            
            
            init_pose_msg.pose.position.x = self.init_pose_list[i][1]
            init_pose_msg.pose.position.y = self.init_pose_list[i][2]
            init_pose_msg.pose.position.z = self.init_pose_list[i][3]
            
            yaw_rad = self.init_pose_list[i][4]
            quat = quaternion_from_euler(0, 0, yaw_rad)
            init_pose_msg.pose.orientation.x = quat[0]
            init_pose_msg.pose.orientation.y = quat[1]
            init_pose_msg.pose.orientation.z = quat[2]
            init_pose_msg.pose.orientation.w = quat[3]
            
            self.init_pose_pub.publish(init_pose_msg)
            rospy.loginfo("Drone {} init pose published".format(i))
            print("Drone {} init pose is: x: {}, y: {}, z: {}, yaw: {}".format(i, self.init_pose_list[i][1], self.init_pose_list[i][2], self.init_pose_list[i][3], self.init_pose_list[i][4]))
            
            if i == self.drone_num-1:
                rospy.loginfo("All drones init pose published")
                rospy.signal_shutdown("All drones init pose published")
            time.sleep(1)

                
if __name__ == '__main__':
    rospy.init_node('init_pose')
    init_pose = InitPosePub()
    time.sleep(1)
    init_pose.publish_init_pose()
    rospy.spin()
    