import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtCore import QThread,pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QGroupBox, QGridLayout, QVBoxLayout, QDesktopWidget, QWidget, QHBoxLayout, QTextBrowser, QComboBox, QLineEdit, QButtonGroup, QRadioButton, QPushButton
from PyQt5.QtGui import QPixmap, QIntValidator, QTextCursor, QIcon
import rospy
from geometry_msgs.msg import PoseStamped
from plotCanvas import PlotCanvas
from nav_msgs.msg import Odometry
from mavlink_acquire import MavlinkAcquire
from ros_acquire import RosAcquire
import time

class SwarmGUI(QWidget):
    def __init__(self,drone_num=1):
        super().__init__()
        self.logo_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../icon/nus.png"))
        self.drone_num = drone_num

        # plot freq control
        # Should initialize params before init_gui
        self.last_plt_time=0
        self.plot_time_gap=0.1
        
        self.init_gui()

    def init_gui(self):
        self.resize(1400, 800)  # 设置窗口大小
        self.center()           # 设置窗口居中
        self.setWindowTitle('NUS Swarm GUI')  # 设置窗口标题
        self.drone_odom_sub_list = []  
        
        connectionGB = self.createConnectionGroupBox()    # Connection
        mapGB = self.createMapGroupBox()                  # Map display
        vehicleInfoGB = self.createVehicleInfoGroupBox()  # swarm info
        # debugGB = self.createDebugGroupBox()              # debug info
        
        # Upper part of the main window 
        funcPart = QWidget()
        sub_layout_1 = QGridLayout()
        sub_layout_1.addWidget(connectionGB, 0, 0)
        sub_layout_1.addWidget(mapGB, 0, 1)
        funcPart.setLayout(sub_layout_1)
        
        # Lower part of the main window
        infoPart = QWidget()
        layout_info = QHBoxLayout()
        layout_info.addWidget(vehicleInfoGB)
        # layout_info.addWidget(debugGB)
        infoPart.setLayout(layout_info)
        
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(funcPart)
        main_layout.addWidget(infoPart)
        self.setLayout(main_layout)
        
        # ROS subscriber initialization
        self.init_subscriber()

        # Start the thread to update the swarm PX4 status
        self.drone_state_thread = DroneStateThread()
        self.drone_state_thread.update_signal.connect(self.updateVehicleInfo)
        self.drone_state_thread.start()

        # Start the thread to update the swarm information from ROS
        self.height_update_thread = HeightUpdateThread()
        self.height_update_thread.height_signal.connect(self.updateHeightInfo)
        self.height_update_thread.start()
    
        self.show()

    # ROS subscriber initialization
    def init_subscriber(self):
        for i in range(self.drone_num):         
            self.map_2d.init_drone_odom_plot(drone_id=i)  
            self.drone_info_label=QLabel(self)
            topic_name=f"/drone_{i}_mavros/local_position/odom"
            self.drone_odom_sub_list.append(rospy.Subscriber(topic_name, Odometry, self.drone_odom_cb,callback_args=i))

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
    def createConnectionGroupBox(self):
        connectionGB = QGroupBox('NUS Swarm GUI')
        
        sub_layout1 = QVBoxLayout()
        
        pixmap = QPixmap(self.logo_path)
        pixmap = pixmap.scaled(300, 150)
        
        logo = QLabel(self)
        logo.setPixmap(pixmap)

        bt1 = QPushButton('Connect formation')
        bt2 = QPushButton('Check formation matrix')
        bt1.clicked.connect(self.connectionButtonClicked)
        bt2.clicked.connect(self.checkButtonClicked)
        
        sub_layout1.addWidget(logo)
        # sub_layout1.addWidget(bt1)
        # sub_layout1.addWidget(bt2)
        # sub_layout1.addWidget(lb3)
        
        connectionGB.setLayout(sub_layout1)
        connectionGB.setFixedSize(300, 200)
        return connectionGB
    
    def connectionButtonClicked(self):
        print('连接按钮被点击')
        
    def checkButtonClicked(self):
        print('检查按钮被点击')
    
    def createMapGroupBox(self):
        mapGB = QGroupBox('map')
        
        sub_layout2 = QVBoxLayout()
        
        # 2D map display
        self.map_2d = PlotCanvas(self, width=5, height=6)

        targetPoseGp = QGroupBox('目标位置')
        layout1 = QHBoxLayout()
        lb1 = QLabel('X')
        self.target_x = QLineEdit()
        lb2 = QLabel('Y')
        self.target_y = QLineEdit()
        lb3 = QLabel('Z')
        self.target_z = QLineEdit()
        
        getTargetPoseBt = QPushButton('获取')
        setTargetPoseBt = QPushButton('设置')
        layout1.addWidget(lb1)
        layout1.addWidget(self.target_x)
        layout1.addWidget(lb2)
        layout1.addWidget(self.target_y)
        layout1.addWidget(lb3)
        layout1.addWidget(self.target_z)
        # layout1.addWidget(getTargetPoseBt)
        # layout1.addWidget(setTargetPoseBt)
        targetPoseGp.setLayout(layout1)
        
        # sub_layout2.addWidget(targetPoseGp)
        
        sub_layout2.addWidget(self.map_2d)  # 增加地图显示
        
        mapGB.setLayout(sub_layout2)
        
        return mapGB
        
    def createVehicleInfoGroupBox(self):
        vehicleInfoGB = QGroupBox('Drone Info')
        
        layout = QGridLayout()
        lb1 = QLabel('Drone ID')
        lb2 = QLabel('State')
        lb3 = QLabel('Kill Switch')
        lb4 = QLabel('Arming')
        lb5 = QLabel('Mode')
        lb6 = QLabel('Battery')
        lb7 = QLabel('Height')
        lb8 = QLabel('VIO Drift')
        layout.addWidget(lb1, 0, 0)
        layout.addWidget(lb2, 1, 0)
        layout.addWidget(lb3, 2, 0)
        layout.addWidget(lb4, 3, 0)
        layout.addWidget(lb5, 4, 0)
        layout.addWidget(lb6, 5, 0)
        layout.addWidget(lb7, 6, 0)
        layout.addWidget(lb8, 7, 0)
        
        # Generate 'self.drone_num' drone info 
        # Drone info：connection status, kill switch, arm state, flight mode, battery, height, and VIO Drift 
        self.infoDisplayLabelList = []
        for i in range (self.drone_num):
            lb1 = QLabel('{}'.format(i))
            layout.addWidget(lb1, 0, i+1)
            le1 = QLabel()
            layout.addWidget(le1, 1, i+1)
            le2 = QLabel()
            layout.addWidget(le2, 2, i+1)
            le3 = QLabel()
            layout.addWidget(le3, 3, i+1)
            le4 = QLabel()
            layout.addWidget(le4, 4, i+1)
            le5 = QLabel()
            layout.addWidget(le5, 5, i+1)
            le6 = QLabel()
            layout.addWidget(le6, 6, i+1)
            le7 = QLabel()
            layout.addWidget(le7, 7, i+1)
            self.infoDisplayLabelList.append([le1, le2, le3, le4, le5, le6, le7])
        
        vehicleInfoGB.setLayout(layout)
        
        return vehicleInfoGB
    
    def updateVehicleInfo(self, drone_status):
        """
        Update the vehicle information 
        from the DroneStateThread
        """

        state = drone_status.get('state')
        battery_info = drone_status.get('battery')

        if state:
            drone_id = state['drone_id']
            system_status = state['system_status']
            armed = state['armed']
            mode = state['flight_mode']
            
            # Connection status
            if drone_id is not None:
                self.infoDisplayLabelList[drone_id][0].setText('Connected')
                self.infoDisplayLabelList[drone_id][0].setStyleSheet("color: green;")
            else:
                self.infoDisplayLabelList[drone_id][0].setText('Disconnected')
                self.infoDisplayLabelList[drone_id][0].setStyleSheet("color: red;") 

            # Kill switch status
            if system_status == 3:
                self.infoDisplayLabelList[drone_id][1].setText('Standby')
                self.infoDisplayLabelList[drone_id][1].setStyleSheet("color: green;")
            elif system_status == 8:
                self.infoDisplayLabelList[drone_id][1].setText('Killed')
                self.infoDisplayLabelList[drone_id][1].setStyleSheet("color: red;")

            # Armed status
            if armed:
                self.infoDisplayLabelList[drone_id][2].setText('Armed')
                self.infoDisplayLabelList[drone_id][2].setStyleSheet("color: green;")
            else:
                self.infoDisplayLabelList[drone_id][2].setText('Disarmed')
                self.infoDisplayLabelList[drone_id][2].setStyleSheet("color: red;") 
            
            # Flight mode
            if mode == 'UNKNOWN':
                self.infoDisplayLabelList[drone_id][3].setText('OFFBOARD')
                self.infoDisplayLabelList[drone_id][3].setStyleSheet("color: green;")
            else:
                self.infoDisplayLabelList[drone_id][3].setText(mode)
                self.infoDisplayLabelList[drone_id][3].setStyleSheet("color: black;")

        if battery_info:
            drone_id = battery_info['drone_id']
            battery = battery_info['battery_remaining']
            
            # Battery status
            self.infoDisplayLabelList[drone_id][4].setText(str(battery))
            if battery >= 60:
                self.infoDisplayLabelList[drone_id][4].setStyleSheet("color: green;")
            elif battery >= 30:
                self.infoDisplayLabelList[drone_id][4].setStyleSheet("color: orange;")
            else:
                self.infoDisplayLabelList[drone_id][4].setStyleSheet("color: red;")

    def updateHeightInfo(self, height_info):
        for drone_id, height in height_info.items():
            self.infoDisplayLabelList[drone_id][5].setText(str(height))

    def createDebugGroupBox(self):
        debugGB = QGroupBox('调试信息')
        
        sub_layout3 = QHBoxLayout()  # 下半区 用来显示信息
        
        self.systemInfo = QTextBrowser()
        sub_layout3.addWidget(self.systemInfo)
        debugGB.setLayout(sub_layout3)
        
        return debugGB

    def drone_odom_cb(self, msg, drone_id):
        # current_time = time.time()
        # if current_time - self.last_plt_time >= self.plot_time_gap:  
        #     self.map_2d.drone_odom_plot((msg.pose.pose.position.x, msg.pose.pose.position.y), drone_id)
        # self.last_plt_time = current_time
        pass

class DroneStateThread(QThread):
    update_signal = pyqtSignal(object)

    def __init__(self):
        super(DroneStateThread, self).__init__()
        self.get_mavlink = MavlinkAcquire('0.0.0.0')
        
    def run(self):
        while True:           
            drone_status = {
                "state": self.get_mavlink.acquire_drone_info(),
                "battery": self.get_mavlink.acquire_drone_battery(),
            }

            valid_status = {k: v for k, v in drone_status.items() if v is not None}
     
            if valid_status:
                self.update_signal.emit(valid_status)
            
            QThread.msleep(200)

class HeightUpdateThread(QThread):
    height_signal = pyqtSignal(object)

    def __init__(self):
        super(HeightUpdateThread, self).__init__()
        self.get_ros = RosAcquire()

    def run(self):
        while True:
            height_info = self.get_ros.acquire_drone_height()
            if height_info:
                self.height_signal.emit(height_info)
            QThread.msleep(10)
            
           
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SwarmGUI(drone_num=4)

    sys.exit(app.exec_())
