'''
TRAN MINH PHUC
2021-24-11   4:05 PM
Test OK
'''

from Libs.DR3_Robotics_Libs import *
link_length = np.array([1,1,1])
#thelta = np.array([60,80,135])
position = np.array([0.5,0.5,1]) #Px Py Pz
Robot = DR3(link_length)
thelta = Robot.Inverse_kinematics(position,1)
print(thelta)
Robot.geometry(thelta,"Geometry Presentation",True)