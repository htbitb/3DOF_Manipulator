import numpy as np
import math as m
import time
from Libs.DR3_Robotics_Libs import *
from DR3_main import *


class Userfunctions(MainWindow):
    def initialize_robot(self, length):
        self.Robot = DR3(length)
        if self.ui.the1_set.text() != "":
            self.the = [
                np.deg2rad(float(self.ui.the1_set.text())),
                np.deg2rad(float(self.ui.the2_set.text())),
                np.deg2rad(float(self.ui.the3_set.text())),
            ]

    def convert_to_Deg(value):
        return np.round(np.rad2deg(value), 2)

    #the1_adjust, the2_adjust, the3_adjust
    def Geometry_display(self):
        try:
            self.the = [
                np.deg2rad(float(self.ui.the1_set.text())),
                np.deg2rad(float(self.ui.the2_set.text())),
                np.deg2rad(float(self.ui.the3_set.text())),
            ]
            self.ui.the1_current.setText(str(Userfunctions.convert_to_Deg(self.the[0])))
            self.ui.the2_current.setText(str(Userfunctions.convert_to_Deg(self.the[1])))
            self.ui.the3_current.setText(str(Userfunctions.convert_to_Deg(self.the[2])))
            T01 = self.Robot.initial_parameters(self.the, 1)
            T02 = self.Robot.initial_parameters(self.the, 2)
            T03 = self.Robot.initial_parameters(self.the, 3)
            T0E = self.Robot.initial_parameters(self.the, 4)
            x = np.array([T01[0, 3], T02[0, 3], T03[0, 3], T0E[0, 3]])
            y = np.array([T01[1, 3], T02[1, 3], T03[1, 3], T0E[1, 3]])
            z = np.array([T01[2, 3], T02[2, 3], T03[2, 3], T0E[2, 3]])

            self.screen.axes.clear()
            self.screen.config_display(self.screen)
            # line -[link length] plot: ve duong line
            self.screen.axes.plot([0, x[0]], [0, y[0]], [-21.6, z[0]], linewidth=9,color='#1f77b4')
            self.screen.axes.plot([x[0], x[1]], [y[0], y[1]], [z[0], z[1]], linewidth=9,color='#1f77b4')
            self.screen.axes.plot([x[1], x[2]], [y[1], y[2]], [z[1], z[2]], linewidth=9, color='#1f77b4')
            self.screen.axes.plot([x[2], x[3]], [y[2], y[3]], [z[2], z[3]], linewidth=9,color='#1f77b4')
            # Joints syntaxis plot: ve dau cham
            self.screen.axes.scatter(0, 0, -21.6, marker="s", color="k", s=300)
            self.screen.axes.scatter(x[0], y[0], z[0], marker="o", color="k", s=200)
            self.screen.axes.scatter(x[1], y[1], z[1], marker="o", color="k", s=200)
            self.screen.axes.scatter(x[2], y[2], z[2], marker="o", color="k", s=200)
            self.screen.axes.scatter(x[3], y[3], z[3], marker="o", color="r", s=200)
            # Hien thi plot
            self.screen.draw()
            # Hien thi vi tri Px Py Pz
            self.ui.current_x.setText(str(np.round(x[3], 2)))
            self.ui.current_y.setText(str(np.round(y[3], 2)))
            self.ui.current_z.setText(str(np.round(z[3], 2)))  
        except:
            print("loading....")

    #btn_reset
    def link_adjustment(self):
        self.link = [
            float(self.length1.text()),
            float(self.length2.text()),
            float(self.length3.text()),
        ]
        self.Robot.ChangeValue(self.link)
        Userfunctions.Geometry_display(self)

    #start_simulation_mode
    def Trajectory(self):
        time = self.ui.time_respond.text().split()
        # print(time)
        compare = int(time[0])
        self.idx = 0
        while (
            self.ui.mode_check.isChecked()
            and self.idx <= compare * 10 - 30 #thoi gian dong co dap ung
            and self.active == True
        ):
            try:
                value_1 = np.round(float(self.ui.the1_set.text()), 2)
                value_2 = np.round(float(self.ui.the2_set.text()), 2)
                value_3 = np.round(float(self.ui.the3_set.text()), 2)
                the = np.array([value_1, value_2, value_3])
                time = self.time_respond.text().split()
                tc = int(time[0])

                a1 = self.the1pre
                b1 = 0
                c1 = 3 * (the[0] - self.the1pre) / tc ** 2
                d1 = -2 * (the[0] - self.the1pre) / tc ** 3

                a2 = self.the2pre
                b2 = 0
                c2 = 3 * (the[1] - self.the2pre) / tc ** 2
                d2 = -2 * (the[1] - self.the2pre) / tc ** 3

                a3 = self.the3pre
                b3 = 0
                c3 = 3 * (the[2] - self.the3pre) / tc ** 2
                d3 = -2 * (the[2] - self.the3pre) / tc ** 3

                self.the1_flex = np.round(
                    a1
                    + b1 * self.idx / 10
                    + c1 * (self.idx / 10) * 2
                    + d1 * (self.idx / 10) * 3,
                    4,
                )
                self.the2_flex = np.round(
                    a2
                    + b2 * self.idx / 10
                    + c2 * (self.idx / 10) * 2
                    + d2 * (self.idx / 10) * 3,
                    4,
                )
                self.the3_flex = np.round(
                    a3
                    + b3 * self.idx / 10
                    + c3 * (self.idx / 10) * 2
                    + d3 * (self.idx / 10) * 3,
                    4,
                )
                self.screen.axes.clear()
                self.screen.config_display(self.screen)
                theta_flex = np.array([self.the1_flex, self.the2_flex, self.the3_flex])
                theta_flex_convert = np.deg2rad(theta_flex)
                self.ui.the1_current.setText(str(theta_flex[0]))
                self.ui.the2_current.setText(str(theta_flex[1]))
                self.ui.the3_current.setText(str(theta_flex[2]))
                T01 = self.Robot.initial_parameters(theta_flex_convert, 1)
                T02 = self.Robot.initial_parameters(theta_flex_convert, 2)
                T03 = self.Robot.initial_parameters(theta_flex_convert, 3)
                T0E = self.Robot.initial_parameters(theta_flex_convert, 4)
                x = np.array([T01[0, 3], T02[0, 3], T03[0, 3], T0E[0, 3]])
                y = np.array([T01[1, 3], T02[1, 3], T03[1, 3], T0E[1, 3]])
                z = np.array([T01[2, 3], T02[2, 3], T03[2, 3], T0E[2, 3]])
                # line -[link length] plot: ve duong line
                self.screen.axes.plot([0, x[0]], [0, y[0]], [-21.6, z[0]], linewidth=9,color='#1f77b4')
                self.screen.axes.plot([x[0], x[1]], [y[0], y[1]], [z[0], z[1]], linewidth=9,color='#1f77b4')
                self.screen.axes.plot([x[1], x[2]], [y[1], y[2]], [z[1], z[2]], linewidth=9, color='#1f77b4')
                self.screen.axes.plot([x[2], x[3]], [y[2], y[3]], [z[2], z[3]], linewidth=9,color='#1f77b4')
                # Joints syntaxis plot: ve dau cham
                self.screen.axes.scatter(0, 0, -21.6, marker="s", color="k", s=300)
                self.screen.axes.scatter(x[0], y[0], z[0], marker="o", color="k", s=200)
                self.screen.axes.scatter(x[1], y[1], z[1], marker="o", color="k", s=200)
                self.screen.axes.scatter(x[2], y[2], z[2], marker="o", color="k", s=200)
                self.screen.axes.scatter(x[3], y[3], z[3], marker="o", color="r", s=200)
                # Hien thi plot
                self.screen.draw()

                self.ui.current_x.setText(str(np.round(x[3], 2)))
                self.ui.current_y.setText(str(np.round(y[3], 2)))
                self.ui.current_z.setText(str(np.round(z[3], 2)))
                self.the1_current.setText(str(round(self.the1_flex, 3)))
                self.the2_current.setText(str(round(self.the2_flex, 3)))
                self.the3_current.setText(str(round(self.the3_flex, 3)))

                self.the1pre = self.the1_flex
                self.the2pre = self.the2_flex
                self.the3pre = self.the3_flex
                self.idx += 1
            except:
                pass

    #btn_start
    def start_process(self):
        self.ui.idx = 0
        self.the1pre = self.the2pre = self.the3pre = 0
        self.ui.the1_current.setText(str(self.the1pre))
        self.ui.the2_current.setText(str(self.the2pre))
        self.ui.the3_current.setText(str(self.the3pre))
        self.the = [
            np.deg2rad(float(self.ui.the1_current.text())),
            np.deg2rad(float(self.ui.the2_current.text())),
            np.deg2rad(float(self.ui.the3_current.text())),
        ]
        self.stop_simulation_mode()
        self.start_simulation_mode()

    def set_joint_angle(self, thelta):
        if np.isnan(thelta[1]) or np.isnan(thelta[2]) or np.isnan(thelta[0]):
            print("Robot is outside of workspace. Please choose another solution")
            print("------------------------------")
            self.ui.out_workspace.setText("Robot is outside of workspace. Please choose another solution")
        else:
            # print("Solution is found")
            Userfunctions.count_solution(self)
            self.ui.out_workspace.setText("")
            thelta1 = str(int(np.rad2deg(thelta[0])))
            thelta2 = str(int(np.rad2deg(thelta[1])))
            thelta3 = str(int(np.rad2deg(thelta[2])))
            self.the1_set.setText(thelta1)
            self.ui.the1_adjust.setValue(int(self.ui.the1_set.text()))
            self.the2_set.setText(thelta2)
            self.ui.the2_adjust.setValue(int(self.ui.the2_set.text()))
            self.the3_set.setText(thelta3)
            self.ui.the3_adjust.setValue(int(self.ui.the3_set.text()))

    #btn_home
    def Home_position(self):
        self.the1_set.setText("0")
        self.ui.the1_adjust.setValue(int(self.ui.the1_set.text()))
        self.the2_set.setText("0")
        self.ui.the2_adjust.setValue(int(self.ui.the2_set.text()))
        self.the3_set.setText("0")
        self.ui.the3_adjust.setValue(int(self.ui.the3_set.text()))
        self.the = [
            np.deg2rad(float(self.ui.the1_set.text())),
            np.deg2rad(float(self.ui.the2_set.text())),
            np.deg2rad(float(self.ui.the3_set.text())),
        ]
        #Tinh toan vi tri EE va lam tron mang
        position = np.round_(self.Robot.Forward_kinematis(self.the), 2)
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

        # Gui gia thi home
        self.time = self.ui.time_respond.text().split()
        self.sum = np.array([0.0,0.0,0.0,int(self.time[0])])
        if(self.ser.isOpen() and self.mode_check.isChecked() == False): 
            self.ser.write('{},{},{},{}'.format(*self.sum).encode())
            Data_send =str('{},{},{},{}'.format(*self.sum))
            self.ser.flushInput()  #flush input buffer, discarding all its contents
            self.ser.flushOutput()
            print(Data_send)

    #btn_for
    def forward_signal(self, step):
        x = float(self.ui.xpos.text()) - step
        y = float(self.ui.ypos.text())
        z = float(self.ui.zpos.text())
        position = [x, y, z]
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        # Chon nghiem
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        # Dieu chinh o QSlider va mo phong o Trajectory 
        Userfunctions.set_joint_angle(self, thelta) 
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

    #btn_back
    def backward_signal(self, step):
        x = float(self.ui.xpos.text()) + step
        y = float(self.ui.ypos.text())
        z = float(self.ui.zpos.text())
        position = [x, y, z]
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        Userfunctions.set_joint_angle(self, thelta)
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

    #btn_left
    def left_signal(self, step):
        x = float(self.ui.xpos.text())
        y = float(self.ui.ypos.text()) - step
        z = float(self.ui.zpos.text())
        position = [x, y, z]
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        Userfunctions.set_joint_angle(self, thelta)
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

    #btn_right
    def right_signal(self, step):
        x = float(self.ui.xpos.text())
        y = float(self.ui.ypos.text()) + step
        z = float(self.ui.zpos.text())
        position = [x, y, z]
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        Userfunctions.set_joint_angle(self, thelta)
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

    #btn_up
    def up_signal(self, step):
        x = float(self.ui.xpos.text())
        y = float(self.ui.ypos.text())
        z = float(self.ui.zpos.text()) + step
        position = [x, y, z]
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        Userfunctions.set_joint_angle(self, thelta)
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

    #btn_down
    def down_signal(self, step):
        x = float(self.ui.xpos.text())
        y = float(self.ui.ypos.text())
        z = float(self.ui.zpos.text()) - step
        position = [x, y, z]
        self.ui.xpos.setText(str(position[0]))
        self.ui.ypos.setText(str(position[1]))
        self.ui.zpos.setText(str(position[2]))
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        Userfunctions.set_joint_angle(self, thelta)
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)

    def change_position_ik(self):
        x = float(self.ui.xpos.text())
        y = float(self.ui.ypos.text())
        z = float(self.ui.zpos.text())
        position = [x, y, z]
        sol = int(self.ui.solution.text())
        thelta = self.Robot.Inverse_kinematics(position, sol)
        Userfunctions.set_joint_angle(self, thelta)
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)
            Userfunctions.send_inverse(self)

    def change_position_fk(self):
        the1 = float(self.ui.the1_set.text())
        the2 = float(self.ui.the2_set.text())
        the3 = float(self.ui.the3_set.text())
        self.ui.the1_adjust.setValue(int(the1))
        self.ui.the2_adjust.setValue(int(the2))
        self.ui.the3_adjust.setValue(int(the3))
        if self.ui.mode_check.isChecked():
            self.stop_simulation_mode()
            # Userfunctions.send(self,position[0],position[1],position[2])
            self.start_simulation_mode()
        else:
            Userfunctions.Geometry_display(self)
            Userfunctions.send_forward(self)

    def count_solution(self):
        x = float(self.ui.xpos.text())
        y = float(self.ui.ypos.text())
        z = float(self.ui.zpos.text())
        count = 0
        position = [x, y, z]
        thelta_1 = self.Robot.Inverse_kinematics(position, 1)
        thelta_2 = self.Robot.Inverse_kinematics(position, 2)
        thelta_3 = self.Robot.Inverse_kinematics(position, 3)
        thelta_4 = self.Robot.Inverse_kinematics(position, 4)
        if np.isnan(thelta_1[1]) or np.isnan(thelta_1[2]) or np.isnan(thelta_1[0]):
            pass
        else:
            print("Solution 1 was found")
            count+=1

        if np.isnan(thelta_2[1]) or np.isnan(thelta_2[2]) or np.isnan(thelta_2[0]):
            pass
        else:
            print("Solution 2 was found")
            count+=1

        if np.isnan(thelta_3[1]) or np.isnan(thelta_3[2]) or np.isnan(thelta_3[0]):
            pass
        else:
            print("Solution 3 was found")
            count+=1

        if np.isnan(thelta_4[1]) or np.isnan(thelta_4[2]) or np.isnan(thelta_4[0]):
            pass
        else:
            print("Solution 4 was found")
            count+=1
        if count == 0:
            print("No solutions were found")
        elif count > 0:
            print("{} solutions were found".format(count))
        print("---------------")
    
    #solu_label: Khi dieu chinh solution thi se tinh toan lai dong hoc nghich
    def change_solution(self):
        solu = self.ui.solution.text()
        # print("Solution is {}".format(solu))
        if solu=='1' or solu=='2' or solu=='3' or solu=='4':
            x = float(self.ui.xpos.text())
            y = float(self.ui.ypos.text())
            z = float(self.ui.zpos.text())
            position = [x, y, z]
            self.ui.xpos.setText(str(position[0]))
            self.ui.ypos.setText(str(position[1]))
            self.ui.zpos.setText(str(position[2]))
            # Chon nghiem
            sol = int(solu)
            thelta = self.Robot.Inverse_kinematics(position, sol)
            # Dieu chinh o QSlider va mo phong o Trajectory 
            Userfunctions.set_joint_angle(self, thelta) 
            if self.ui.mode_check.isChecked():
                self.stop_simulation_mode()
                self.start_simulation_mode()
            else:
                Userfunctions.Geometry_display(self)
        elif solu=='':
            pass
        else:
            pass


    def send_ik(self,x,y,z):
        # self.time = self.time_respond.value()
        self.time = self.ui.time_respond.text().split()
    #    self.xpos.setText(str(x))
    #    self.ypos.setText(str(y))
    #    self.zpos.setText(str(z))
        position = [x, y, z]
        sol = int(self.ui.solution.text())
        self.setthe = self.Robot.Inverse_kinematics(position, sol)
        self.setthe_deg = Userfunctions.convert_to_Deg(self.setthe)
    #    self.setthe = np.round_(UIFunctions.inverse_kinematic(x,y,z),1)
        self.sum = np.array([self.setthe_deg[0],self.setthe_deg[1],self.setthe_deg[2],int(self.time[0])])
        if(self.ser.isOpen() and self.mode_check.isChecked() == False):
            self.ser.write('{},{},{},{}'.format(*self.sum).encode())
            Data_send =str('{},{},{},{}'.format(*self.sum))
            # self.ser.flushInput()  #flush input buffer, discarding all its contents
            # self.ser.flushOutput()
            print(Data_send)


    def send_fk(self,the1,the2,the3):
        self.time = self.ui.time_respond.text().split()
        self.sum = np.array([the1,the2,the3,int(self.time[0])])
        if(self.ser.isOpen() and self.mode_check.isChecked() == False):
            self.ser.write('{},{},{},{}'.format(*self.sum).encode())
            Data_send =str('{},{},{},{}'.format(*self.sum))
            self.ser.flushInput()  #flush input buffer, discarding all its contents
            self.ser.flushOutput()
            print(Data_send)

    # def send_home(self):
    #     # Home Position
    #     x = -11.8
    #     y = 0.0
    #     z = 13.1

    #     self.stop_worker_mode()
    #     Userfunctions.send(self,x,y,z)
    #     self.start_worker_mode()
        
    def send_inverse(self):
        set_x = float(self.xpos.text())
        set_y = float(self.ypos.text())
        set_z = float(self.zpos.text())
        self.stop_simulation_mode()
        Userfunctions.send_ik(self,set_x,set_y,set_z)
        self.start_simulation_mode()

    def send_forward(self):
        the1 = float(self.ui.the1_adjust.value())
        the2 = float(self.ui.the2_adjust.value())
        the3 = float(self.ui.the3_adjust.value())
        self.stop_simulation_mode()
        Userfunctions.send_fk(self,the1,the2,the3)
        self.start_simulation_mode()

    # def receive(self):
    #     count = 0
    #     while self.ser.isOpen() and count<150:
    #         strdata = self.ser.readline().decode()
    #         self.ser.flushInput()
    #         sys.stdout.write('\r'+strdata)
    #         # print(strdata)
    #         count+=1
