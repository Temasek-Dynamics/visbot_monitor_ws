#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from controller_msgs.msg import PoseStamped as PoseStampedController

def goal_callback(msg):
    new_msg = PoseStampedController()
    new_msg.header = msg.header
    new_msg.pose = msg.pose
    new_msg.drone_id = -1
    pub.publish(new_msg)


if __name__ == '__main__':
    rospy.init_node('goal_pose_remap')
    pub = rospy.Publisher('/goal', PoseStampedController, queue_size=10)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    rospy.spin()