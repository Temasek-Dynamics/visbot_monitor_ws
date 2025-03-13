#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import tf2_ros
from cv_bridge import CvBridge
import numpy as np
import tf.transformations
from geometry_msgs.msg import TransformStamped

def depthCallback(msg):
    image = bridge.imgmsg_to_cv2(msg, "32FC1")
    valid_pixels = image[image > 0.1]
    
    if valid_pixels.size > 0:
        print("Min:", np.min(valid_pixels), "Max:", np.max(valid_pixels))
    else:
        print("No valid pixels above threshold")


def staticTrans(frame, child_frame, trans, euler):
    tfs = TransformStamped()
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = frame
    tfs.child_frame_id = child_frame
    tfs.transform.translation.x = trans[0]
    tfs.transform.translation.y = trans[1]
    tfs.transform.translation.z = trans[2]
    qtn = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    return tfs


if __name__ == '__main__':
    rospy.init_node('tf_transform', anonymous=False)
    rospy.loginfo('Running until shutdown (Ctrl-C).')
    bridge = CvBridge()
    trans = [0, 0, 0]
    euler = [np.deg2rad(-105), 0, np.deg2rad(-90)]
    tf_pub = tf2_ros.StaticTransformBroadcaster()
    STATIC_TRANS = staticTrans('map', 'camera_link', trans, euler)
    tf_pub.sendTransform(STATIC_TRANS)

    rospy.Subscriber('/depth_topic', Image, depthCallback)
    rospy.spin()