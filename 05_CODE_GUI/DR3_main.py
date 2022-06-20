import sys
import matplotlib
import platform
import os
import time
import serial.tools.list_ports
import serial
import numpy as np

from matplotlib import cm
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.ticker as ticker
from matplotlib.animation import FuncAnimation

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import (QCoreApplication, QPropertyAnimation, QDate, QDateTime, QMetaObject, QObject, QPoint, QRect, QSize, QTime, QUrl, Qt, QEvent)
from PyQt5.QtGui import (QBrush, QColor, QConicalGradient, QCursor, QFont, QFontDatabase, QIcon, QKeySequence, QLinearGradient, QPalette, QPainter, QPixmap, QRadialGradient)
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot

import warnings
warnings.filterwarnings('ignore')
from app_modules import *
class Background(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = uic.loadUi('Display_setting/DR3_Background.ui',self)
        self.setWindowIcon(QtGui.QIcon('robotics.png'))
        self.setWindowTitle('DR3 Controller')
        self.ui.btn_start.clicked.connect(self.show_screen)
    def show_screen(self):
        self.main = MainWindow()
        self.main.show()
        self.close()
        
class Display(FigureCanvas):
    def __init__(self,parent=None, width = 70, height = 50,dpi=75):
        figure = Figure(figsize=(width,height),dpi=dpi)
        figure.patch.set_facecolor('#343b48')
        figure.suptitle('3D Robotics Simulation',color='white',fontsize=15)
        # setting style sheets
        # style.use("seaborn-notebook")

        # creat 3D instance
        # self.axes = figure.add_subplot(111,projection='3d')
        self.axes = figure.gca(projection='3d')
        # adjust the subplot params
        figure.tight_layout()
        super(Display, self).__init__(figure)
    def config_display(self,widget):
        widget.axes.set_facecolor('#343b48')
        widget.axes.grid(True)
        widget.axes.set_xlim(-40,40)
        widget.axes.set_ylim(-40, 40)
        widget.axes.set_zlim(-25, 40)

        widget.axes.set_xlabel('X_axis',color='white',fontsize=10)
        widget.axes.set_ylabel('Y_axis',color='white',fontsize=10)
        widget.axes.set_zlabel('Z_axis',color='white',fontsize=10)
        widget.axes.tick_params(axis='x', colors='white')
        widget.axes.tick_params(axis='y', colors='white')
        widget.axes.tick_params(axis='z', colors='white')
class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = uic.loadUi('Display_setting/DR3_Display.ui',self)
        self.setWindowIcon(QtGui.QIcon('robotics.png'))
        self.setWindowTitle('DR3 Controller')
        #self.ui.setupUi(self)
        print('System: '+platform.system())
        print('Version: '+platform.release())
        self.ser =  serial.Serial()
        UIFunctions.uiDefinitions(self)

        ## initializing Threading Core
        self.threadpool = QtCore.QThreadPool()
        self.threadpool_streamdata = QtCore.QThreadPool()
        self.threadpool_read = QtCore.QThreadPool()

        ## initialize parameter:
        self.link = [float(self.length1.text()),
                     float(self.length2.text()),
                     float(self.length3.text())]
        Userfunctions.initialize_robot(self,self.link)

        # Khoi tao man hinh plot robot
        self.screen = Display(self,width=50, height=50, dpi=70)
        self.screen.config_display(self.screen)
        self.ui.screen_form.addWidget(self.screen)      #QFormLayout

        self.ui.btn_Setting.clicked.connect(lambda: UIFunctions.toggleMenu_setting(self,280,True))
        self.ui.the1_adjust.valueChanged.connect(lambda: UIFunctions.valuechange(self))
        self.ui.the1_adjust.valueChanged.connect(lambda: Userfunctions.Geometry_display(self))
        self.ui.the2_adjust.valueChanged.connect(lambda: UIFunctions.valuechange(self))
        self.ui.the2_adjust.valueChanged.connect(lambda: Userfunctions.Geometry_display(self))
        self.ui.the3_adjust.valueChanged.connect(lambda: UIFunctions.valuechange(self))
        self.ui.the3_adjust.valueChanged.connect(lambda: Userfunctions.Geometry_display(self))

        # self.ui.xpos.solution.connect(lambda: Userfunctions.change_solution(self))
        # self.ui.xpos.textChanged.connect(lambda: Userfunctions.change_position(self))
        # self.ui.ypos.textChanged.connect(lambda: Userfunctions.change_position(self))
        # self.ui.zpos.textChanged.connect(lambda: Userfunctions.change_position(self))
        
        # ARDUINO BTN: TEST LAI
        self.btnconnect_arduino.clicked.connect(lambda: UIFunctions.connect_arduino_clicked(self))
        self.btn_disconnect_arduino.clicked.connect(lambda: UIFunctions.disconnect_arduino_clicked(self))

        self.ui.btn_plus.clicked.connect(lambda: UIFunctions.timechange_plus(self))
        self.ui.btn_minus.clicked.connect(lambda: UIFunctions.timechange_minus(self))
        
        self.ui.btn_reset.clicked.connect(lambda: Userfunctions.link_adjustment(self))
        self.ui.btn_home.clicked.connect(lambda: Userfunctions.Home_position(self))

        ## button Functionality of Inverse Kinematics: step = 5
        self.ui.btn_left.clicked.connect(lambda: Userfunctions.left_signal(self,0.5))
        self.ui.btn_right.clicked.connect(lambda: Userfunctions.right_signal(self,0.5))
        self.ui.btn_for.clicked.connect(lambda: Userfunctions.forward_signal(self,0.5))
        self.ui.btn_back.clicked.connect(lambda: Userfunctions.backward_signal(self,0.5))
        self.ui.btn_up.clicked.connect(lambda: Userfunctions.up_signal(self,0.5))
        self.ui.btn_down.clicked.connect(lambda: Userfunctions.down_signal(self,0.5))
        self.ui.btn_sending_fk.clicked.connect(lambda: Userfunctions.send_forward(self))
        # self.ui.btn_sending_fk.clicked.connect(lambda: Userfunctions.receive(self))
        self.ui.btn_sending_ik.clicked.connect(lambda: Userfunctions.send_inverse(self))

        ## button simulation mode
        self.ui.btn_start.clicked.connect(lambda: Userfunctions.start_process(self))
        self.ui.mode_check.stateChanged.connect(lambda: UIFunctions.simulation_check(self))

        ## Realtime Display Event:
        #self.timer = QtCore.QTimer()
        #self.timer.timeout.connect(lambda: Userfunctions.Geometry_display(self))
        #self.timer.start(100)

        ## Mouse Clicked and Keyboard Event ##
    def mousePressEvent(self, event):
        self.dragPos = event.globalPos()
        if event.buttons() == Qt.RightButton:
            pass
        if event.buttons() == Qt.MidButton:
            UIFunctions.Update_value(self)

    def keyPressEvent(self, event):
        print('Key: ' + str(event.key()) + ' | Text Press: ' + str(event.text()))
        if event.key() == Qt.Key_R:
            UIFunctions.reset(self)
        if event.key() == Qt.Key_A:
            Userfunctions.change_position_ik(self)
        if event.key() == Qt.Key_M:
            Userfunctions.change_position_fk(self)
        if event.key() == Qt.Key_S:
            Userfunctions.change_solution(self)

    def start_simulation_mode(self):
        self.active = True
        worker = Worker(lambda: Userfunctions.Trajectory(self))
        self.threadpool.start(worker)

    def stop_simulation_mode(self):
        self.active = False
        time.sleep(0.3)

    def start_worker_streamdata(self):
        worker_streamdata = Worker(self.start_stream_data_arduino, )
        self.threadpool_streamdata.start(worker_streamdata)

    def start_worker_read(self):
        worker_read = Worker(self.read, )
        self.threadpool_read.start(worker_read)	

    def start_stream_data_arduino(self):
        while  self.ser.isOpen() and self.mode_check.isChecked() == False:
            try:
                #readline: doc dong dau tien gui ve
                #decode: giai ma chuoi string
                strdata = self.ser.readline().decode()
                time.sleep(0.1)
                self.theta = strdata.split()
                self.position = UIFunctions.Forward_kinematis(self.theta)
                self.current_x.setText(str(round(self.position[0],3)))
                self.current_y.setText(str(round(self.position[1],3)))
                self.current_z.setText(str(round(self.position[2],3)))
                if len(self.theta) == 3:
                    self.the1_current.setText(self.theta[0])
                    self.the2_current.setText(self.theta[1])
                    self.the3_current.setText(self.theta[2])
            except:
                pass


    def read(self):
        while self.ser.isOpen():
            try:
                T01 = self.Robot.initial_parameters(self.theta, 1)
                T02 = self.Robot.initial_parameters(self.theta, 2)
                T03 = self.Robot.initial_parameters(self.theta, 3)
                T0E = self.Robot.initial_parameters(self.theta, 4)
                x = np.array([T01[0, 3], T02[0, 3], T03[0, 3], T0E[0, 3]])
                y = np.array([T01[1, 3], T02[1, 3], T03[1, 3], T0E[1, 3]])
                z = np.array([T01[2, 3], T02[2, 3], T03[2, 3], T0E[2, 3]])

                self.screen.axes.clear()
                self.screen.config_display(self.screen)
                # line -[link length] plot: ve duong line
                self.screen.axes.plot([0, x[0]], [0, y[0]], [-10, z[0]], linewidth=9,color='#1f77b4')
                self.screen.axes.plot([x[0], x[1]], [y[0], y[1]], [z[0], z[1]], linewidth=9,color='#1f77b4')
                self.screen.axes.plot([x[1], x[2]], [y[1], y[2]], [z[1], z[2]], linewidth=9, color='#1f77b4')
                self.screen.axes.plot([x[2], x[3]], [y[2], y[3]], [z[2], z[3]], linewidth=9,color='#1f77b4')
                # Joints syntaxis plot: ve dau cham
                self.screen.axes.scatter(0, 0, -10, marker="s", color="k", s=300)
                self.screen.axes.scatter(x[0], y[0], z[0], marker="o", color="k", s=200)
                self.screen.axes.scatter(x[1], y[1], z[1], marker="o", color="k", s=200)
                self.screen.axes.scatter(x[2], y[2], z[2], marker="o", color="k", s=200)
                self.screen.axes.scatter(x[3], y[3], z[3], marker="o", color="r", s=200)
                # Hien thi plot
                self.screen.draw()
            except:
                pass


class Worker(QtCore.QRunnable):

	def __init__(self, function, *args, **kwargs):
		super(Worker, self).__init__()
		self.function = function
		self.args = args
		self.kwargs = kwargs

	@pyqtSlot()
	def run(self):

		self.function(*self.args, **self.kwargs)


if __name__=="__main__":
    app = QApplication(sys.argv)
    window = Background()
    window.show()
    sys.exit(app.exec_())
