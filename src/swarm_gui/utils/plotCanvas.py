'''
Author: Lei He
Date: 2024-05-23 12:44:42
LastEditTime: 2024-05-23 12:44:46
Description: map visulization for swarm gui
Github: https://github.com/heleidsn
'''

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        # Initial map and drone plot
        self.map_plot()           
        self.drone_plots = {}  # Store the drone plots
        self.drone_annotations = {}  # Store the drone annotations
        self.background = None  # Store the background for blitting
    
    def init_drone_odom_plot(self,drone_id):
        self.drone_plots[drone_id], = self.axes.plot([], [], 'ro')  # Initialize an empty plot for the drone position
        self.drone_annotations[drone_id] = self.axes.annotate('', xy=(0, 0), xytext=(0, -2))
    
    def drone_odom_plot(self, drone_position,drone_id):
        

        x, y = drone_position
        self.drone_plots[drone_id].set_data([x], [y])  # Update the data of the plot

        # 更新注释位置
        annotation = self.drone_annotations[drone_id]
        annotation.set_text(f'Drone {drone_id} ({x:.2f}, {y:.2f})')
        annotation.set_position((x, y - 2))
        annotation.xy = (x, y)

        self.draw()
        
    def map_plot(self):
        self.axes.set_title('Drone Position')
        self.axes.set_xlabel('X')
        self.axes.set_ylabel('Y')
        self.axes.set_xlim(-0, 20)
        self.axes.set_ylim(-10, 10)
        self.axes.grid(True)
        self.draw_idle()


##################################################################################################
###################################################################################################
# import sys
# import random
# from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
# from PyQt5.QtCore import QTimer
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.figure import Figure
# from matplotlib.animation import FuncAnimation

# class PlotCanvas(FigureCanvas):
#     def __init__(self, parent=None, width=5, height=4, dpi=100):
#         fig = Figure(figsize=(width, height), dpi=dpi)
#         self.axes = fig.add_subplot(111)

#         FigureCanvas.__init__(self, fig)
#         self.setParent(parent)

#         # Initial map plot
#         self.map_plot()
#         self.drone_plots = {}  # Store the drone plots
#         self.drone_annotations = {}  # Store the drone annotations

#         # Set up the animation
#         self.anim = FuncAnimation(fig, self.update_animation, frames=self.generate_frames, blit=True, interval=1000)

#     def init_drone_odom_plot(self, drone_id):
#         self.drone_plots[drone_id], = self.axes.plot([], [], 'ro')  # Initialize an empty plot for the drone position
#         self.drone_annotations[drone_id] = self.axes.annotate('', xy=(0, 0), xytext=(0, -2))

#     def drone_odom_plot(self, drone_position, drone_id):
#         x, y = drone_position
#         self.drone_plots[drone_id].set_data([x], [y])  # Update the data of the plot

#         # Update annotation position and text
#         annotation = self.drone_annotations[drone_id]
#         annotation.set_text(f'Drone {drone_id} ({x:.2f}, {y:.2f})')
#         annotation.set_position((x, y - 2))
#         annotation.xy = (x, y)

#     def map_plot(self):
#         self.axes.set_title('Drone Position')
#         self.axes.set_xlabel('X')
#         self.axes.set_ylabel('Y')
#         self.axes.set_xlim(-0, 20)
#         self.axes.set_ylim(-10, 10)
#         self.axes.grid(True)

#     def generate_frames(self):
#         # Generate dummy data for animation frames
#         for _ in range(10):  # Generate 10 frames
#             yield [(random.uniform(0, 20), random.uniform(-10, 10), i) for i in range(1, 4)]  # (x, y, drone_id)

#     def update_animation(self, frames):
#         for frame in frames:
#             for x, y, drone_id in frame:
#                 if drone_id not in self.drone_plots:
#                     self.init_drone_odom_plot(drone_id)
#                 self.drone_odom_plot((x, y), drone_id)

#         return list(self.drone_plots.values()) + list(self.drone_annotations.values())

# class App(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.initUI()

#     def initUI(self):
#         self.setWindowTitle('Drone Position Animation')
#         self.setGeometry(100, 100, 800, 600)

#         widget = QWidget(self)
#         self.setCentralWidget(widget)
#         layout = QVBoxLayout(widget)

#         self.m = PlotCanvas(self, width=5, height=4)
#         layout.addWidget(self.m)

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     ex = App()
#     ex.show()
#     sys.exit(app.exec_())