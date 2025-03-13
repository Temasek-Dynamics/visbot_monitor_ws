import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class RosAcquire:
    def __init__(self):       
        self.drone_num = 12
        self.drone_odom_sub_list = []
        self.odom_info_dict = {i: None for i in range(self.drone_num)}
        self.height_info_dict = {i: None for i in range(self.drone_num)}

        self.init_subscriber()
        
    def init_subscriber(self):
        for i in range(self.drone_num):           
            topic_name=f"/drone_{i}_mavros/local_position/odom"
            rospy.Subscriber(topic_name, Odometry, self.heightCallback,callback_args=i)
            # rospy.Subscriber(topic_name, Odometry, self.odomCallback,callback_args=i)

    # def odomCallback(self, msg, drone_id):
    #     self.odom_info_dict[drone_id] = {
    #         'x': round(msg.pose.pose.position.x, 6),
    #         'y': round(msg.pose.pose.position.y, 6),
    #         'height': round(msg.pose.pose.position.z, 6)
    #     }

    def heightCallback(self, msg, drone_id):
        self.height_info_dict[drone_id] = round(msg.pose.pose.position.z, 6)

    def acquire_drone_height(self):
        return {drone_id: info for drone_id, info in self.height_info_dict.items() if info is not None}
    
    # def acquire_drone_odom(self):
    #     return {drone_id: info for drone_id, info in self.odom_info_dict.items() if info is not None}