'''
Author: Lei He
Date: 2024-05-23 12:44:42
LastEditTime: 2024-05-23 13:05:10
Description: ROS node for swarm GUI
Github: https://github.com/heleidsn
'''
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
class SwarmNode:
    def __init__(self):
        
        self.drone_num = 0
        self.drone_odom_sub_list = []
        self.init_ros_node()
        
    def init_ros_node(self):
        rospy.init_node('gcs_gui', anonymous=True)

        for i in range(self.drone_num):           
            topic_name=f"/drone_{i}_mavros/local_position/odom"
            self.drone_odom_sub_list.append(rospy.Subscriber(topic_name, Odometry, self.drone_odom_cb,callback_args=i))
    
    def drone_odom_cb(self, msg, drone_id):
        pass

    def update_swarm_info(self):
        pass

    def update_debug_info(self):
        pass

    def update_map(self):
        pass

    def update_vehicle_info(self):
        pass