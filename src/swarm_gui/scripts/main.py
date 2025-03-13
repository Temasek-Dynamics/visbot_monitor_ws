#!/usr/bin/env python3

'''
Author: Lei He
Date: 2024-05-23 10:17:08
LastEditTime: 2024-05-23 12:48:34
Description: nus_formation_gui
Github: https://github.com/heleidsn
'''

import os
import sys
import rospy
from PyQt5.QtWidgets import QApplication

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from utils.mainWindow import SwarmGUI

if __name__ == "__main__":
    rospy.init_node('swarm_gui', anonymous=True)
    drone_num = rospy.get_param('~drone_num', 4)

    app = QApplication(sys.argv)
    gui = SwarmGUI(drone_num=drone_num)

    try:
        sys.exit(app.exec_())
    finally:
        os.system("rosnode kill /swarm_gui_node")
        if "/commander_ros_proxy_gui" in os.popen("rosnode list").read():
            os.system("rosnode kill /commander_ros_proxy_gui")