'''
TRAN MINH PHUC
2021-24-11   4:05 PM
Test OK
'''

import numpy as np
import math  as m
import sympy as sym
import matplotlib.pyplot as plt

c = lambda x: np.cos(x)
s = lambda x: np.sin(x)
t = lambda x,y: m.atan2(x,y)

class DR3:
    def __init__(self,link_length):
        self.l = link_length

    def ChangeValue(self,new_length):
        self.l = new_length

    def DHmatrix(self,alp,a,d,the):
        '''
        the is called the joint variable
        d is the joint variable
        The geometry of a robotic mechanism is conveniently defined
        by attaching coordinate frames to each link.While these frames
        could be located arbitrarily, it is advantageous both for
        consistency and computational efficiency to
        adhere to a convention for locating the frameson the links.
        '''
        Mdh=np.matrix([[c(the)        , -s(the)       , 0       , a],
                       [s(the)*c(alp) , c(the)*c(alp) ,-s(alp)  ,-d*s(alp)],
                       [s(the)*s(alp) , c(the)*s(alp) , c(alp)  , d*c(alp)],
                       [0             ,      0        , 0       , 1]])
        return Mdh

    def initial_parameters(self,the,option = 1):
        '''
        DHMATRIX Summary of this function goes here
         i      alpha(i-1)          di           a(i-1)        Thelta(i)
       joint    Link twist     Link Offset    Link Leight   Joint variable
         1       0                 0             L1             the1;...       %frame1
         2       pi/2              0             0              the2;...       %frame2
         3       0                 L2            0              the3;...       %frame3
        end      0                 L3            0                0];          %End-point

         Joint frame with respect to the world coordinates.
        '''
        T01 = np.array([[c(the[0]), -s(the[0]), 0,          0],
                        [s(the[0]),  c(the[0]), 0,          0],
                        [0        ,          0, 1,          0],
                        [0        ,          0, 0,          1]])

        T12 = np.array([[c(the[1]), -s(the[1]),  0,  self.l[0]],
                        [0        ,          0,  1,         0],
                        [s(the[1]),  c(the[1]),  0,         0],
                        [0        ,          0,  0,         1]])

        T23 = np.array([[c(the[2]), -s(the[2]),  0,  self.l[1]],
                        [s(the[2]),  c(the[2]),  0,        0],
                        [0        ,          0,  1,        0],
                        [0        ,          0,  0,        1]])

        T3E = np.array([[1,0, 0,  self.l[2]],
                        [0,1, 0,         0],
                        [0,0, 1,         0],
                        [0,0, 0,         1]])
        # Tranformation Matrix:
        T02 = np.dot(T01,T12)
        T03 = np.dot(T02,T23)
        T0E = np.dot(T03,T3E)
        if   option == 1:
            return T01
        elif option == 2:
            return T02
        elif option == 3:
            return T03
        elif option == 4:
            return T0E

    def Forward_kinematis(self,the):
        End_Effector_Frame = self.initial_parameters(the,4)
        x = End_Effector_Frame[0,3]
        y = End_Effector_Frame[1,3]
        z = End_Effector_Frame[2,3]
        position = np.array([x,y,z])
        return position
        
    def basic_01(self,a,b,d):
        anp = t(b,a)
        x1 = anp + t(np.sqrt(a*a + b*b - d*d),d)
        x2 = anp + t(-np.sqrt(a*a + b*b - d*d),d)
        x = np.array([x1,
                      x2])
        return x

    def Inverse_kinematics(self,pos,solution=1):
        #scenario 1: Px = pos[0]; Py = pos[0]; Pz = pos[0];
        the1_1 = t(pos[1],pos[0])
        a3_1 = 2*self.l[1]*self.l[2]
        b3_1 = 0
        d3_1 = (pos[0]*c(the1_1) + pos[1]*s(the1_1) - self.l[0])**2 + pos[2]**2 - self.l[1]**2 - self.l[2]**2
        the3_1 = self.basic_01(a3_1,b3_1,d3_1)
        a_1 = self.l[2]*s(the3_1)
        b_1 = self.l[2]*c(the3_1) + self.l[1]
        c_1 = pos[0]*c(the1_1) + pos[1]*s(the1_1) - self.l[0]
        d_1 = pos[2]
        s2_1 = (d_1 - a_1*c_1/b_1) / (a_1*a_1/b_1 + b_1)
        c2_1 = (d_1 + c_1*b_1/a_1) / (b_1*b_1/a_1 + a_1)
        the2_1 = t(s2_1[0],c2_1[0])
        the2_2 = t(s2_1[1],c2_1[1])
        #scenario 2:
        the1_2 = t(-pos[1],-pos[0])
        a3_2 = 2*self.l[1]*self.l[2]
        b3_2 = 0
        d3_2 = (pos[0]*c(the1_2) + pos[1]*s(the1_2) - self.l[0])**2 + pos[2]**2 - self.l[1]**2 - self.l[2]**2
        the3_2 = self.basic_01(a3_2,b3_2,d3_2)
        a_2 = self.l[2]*s(the3_2)
        b_2 = self.l[2]*c(the3_2) + self.l[1]
        c_2 = pos[0]*c(the1_2) + pos[1]*s(the1_2) - self.l[0]
        d_2 = pos[2]
        s2_2 = (d_2 - a_2*c_2/b_2) / (a_2*a_2/b_2 + b_2)
        c2_2 = (d_2 + c_2*b_2/a_2) / (b_2*b_2/a_2 + a_2)
        the2_3 = t(s2_2[0],c2_2[0])
        the2_4 = t(s2_2[1],c2_2[1])
        print(np.rad2deg(the1_1))
        print(np.rad2deg(the2_1))
        print(np.rad2deg(the3_1[0]))
        if solution == 1:
            if -135<np.rad2deg(the1_1)<135 and -20<np.rad2deg(the2_1)<150 and -90<np.rad2deg(the3_1[0])<135:
                Sol1 = np.array([the1_1   ,  the2_1  ,   the3_1[0]])
            else:
                Sol1 = np.array([np.nan   ,  np.nan  ,   np.nan])
            return Sol1

        elif solution == 2:
            if -135<np.rad2deg(the1_1)<135 and -20<np.rad2deg(the2_2)<150 and -90<np.rad2deg(the3_1[1])<135:
                Sol2 = np.array([the1_1   ,  the2_2  ,   the3_1[1]])
            else:
                Sol2 = np.array([np.nan   ,  np.nan  ,   np.nan])
            return Sol2

        elif solution == 3:
            if -135<np.rad2deg(the1_2)<135 and -20<np.rad2deg(the2_3)<150 and -90<np.rad2deg(the3_2[0])<135:
                Sol3 = np.array([the1_2   ,  the2_3  ,   the3_2[0]])
            else:
                Sol3 = np.array([np.nan   ,  np.nan  ,   np.nan])
            return Sol3

        elif solution == 4:
            if -135<np.rad2deg(the1_2)<135 and -20<np.rad2deg(the2_4)<150 and -90<np.rad2deg(the3_2[1])<135:
                Sol4 = np.array([the1_2   ,  the2_4  ,   the3_2[1]])
            else:
                Sol4 = np.array([np.nan   ,  np.nan  ,   np.nan])
            return Sol4

    def geometry(self,the,title=" ",enable = False):
        T01 = self.initial_parameters(the,1)
        T02 = self.initial_parameters(the,2)
        T03 = self.initial_parameters(the,3)
        T0E = self.initial_parameters(the,4)
        x = np.array([T01[0,3],T02[0,3],T03[0,3],T0E[0,3]])
        y = np.array([T01[1,3],T02[1,3],T03[1,3],T0E[1,3]])
        z = np.array([T01[2,3],T02[2,3],T03[2,3],T0E[2,3]])
        plt.ion()
        plt.style.use("seaborn-notebook")
        fig = plt.figure()
        axis = fig.add_subplot(111,projection='3d')

        # line -[link length] plot
        axis.plot([0,x[0]],[0,y[0]],[0,z[0]],linewidth=5)
        axis.plot([x[0],x[1]],[y[0],y[1]],[z[0],z[1]],linewidth=5)
        axis.plot([x[1],x[2]],[y[1],y[2]],[z[1],z[2]],linewidth=5)
        axis.plot([x[2],x[3]],[y[2],y[3]],[z[2],z[3]],linewidth=5)
        # Joints syntaxis plot
        axis.scatter(0, 0, 0, color='black',linewidth=8)
        axis.scatter(x[0], y[0], z[0], color='black',linewidth=7)
        axis.scatter(x[1], y[1], z[1], color='black',linewidth=7)
        axis.scatter(x[2], y[2], z[2], color='black',linewidth=7)
        axis.scatter(x[3], y[3], z[3], color='red',linewidth=7)
        if enable:
            label = '  ({:.1f},{:.1f},{:.1f})' .format(x[3], y[3],z[3])
            axis.text(x[3],y[3],z[3],label,fontsize=7,color='red')
        axis.set_xlim(-3, 8)
        axis.set_ylim(-3, 8)
        axis.set_zlim(-3, 8)
        axis.set_xlabel('X axis')
        axis.set_ylabel('Y axis')
        axis.set_zlabel('Z axis')
        plt.title(title)
