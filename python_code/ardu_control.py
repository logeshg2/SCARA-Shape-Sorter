#!/usr/bin/python3
import time
import serial
from numpy import *
from numpy import round as np_round

# links
L1 = 228  # L1 = 228mm
L2 = 136.5 # L2 = 136.5mm

# *********** WIP **********
# limit setting ( the max or min movement of the links and z movement)
# apply error and threshold for the output from IK
# Still joint3 angle and Z is not defined with IK

class arm_control:
    def __init__(self,serialport='/dev/ttyACM0'):
        self.ser_port = serial.Serial(port = serialport, baudrate=115200, timeout=3)
        self.wait_ser()
        self.wait_homing()

        # variables
        self.saveStatus = 0
        self.flip = 0
        self.j1Slider = 0
        self.j2Slider = 0
        self.j3Slider = 0
        self.zSlider = 200
        self.servoValue = 0              # Servo value -> 0 to 180 ; > initially at (0 - open) and (180 - close)
        self.speedSlider = 500
        self.accelerationSlider = 500
        self.data = ""

        self.skip = False

        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.z = self.zSlider - 65       # self.z is the distance from base to the end-effector z height
                                         # 65 is the distance to be reduced (***IMP***)              


    def wait_ser(self): # serial confirmation
        while (self.ser_port.in_waiting == 0):
            continue
        data_r = self.ser_port.readline().decode()
        print(data_r) 
    
    def wait_homing(self): # wait untill homing is done
        while (self.ser_port.in_waiting == 0):
            continue
        data_r = self.ser_port.readline().decode()
        if data_r == "Homed": # checking for homing 
            print(data_r)            

    def write_wait(self): # wait's and returns the data
        self.ser_port.write(self.data.encode()) # writing data 
        while (self.ser_port.in_waiting == 0):
            continue
        data_r = self.ser_port.readline().decode()
        if data_r == self.data: # print confirmation
            print(data_r)


    def forwardKinematics(self):
        theta1_rad = deg2rad(self.theta1)# * pi / 180
        theta2_rad = deg2rad(self.theta2)# * pi / 180
        xP = round(L1 * cos(theta1_rad) + L2 * cos(theta1_rad + theta2_rad))
        yP = round(L1 * sin(theta1_rad) + L2 * sin(theta1_rad + theta2_rad))
        # Current end-effector point is (xP,yP)
        return (xP,yP)

    # Inverse Kinematics (*** IWP *** for joint 3 and Z)
    def inverseKinematics(self,x,y):
        r1 = sqrt(x**2+y**2)  # eqn 1
        phi_1 = arccos((L2**2 - L1**2 - r1**2) / (-2 * L1 * r1))  # eqn 2
        phi_2 = arctan2(y, x)  # eqn 3
        self.theta1 = rad2deg(phi_2-phi_1)  # eqn 4 converted to degrees

        phi_3 = arccos((r1**2 - L1**2 - L2**2) / (-2 * L1 * L2))
        self.theta2 = 180-rad2deg(phi_3)

        # added (28.11.2023)
        # Adjust angles for the fourth quadrant
        # if x <= 0 and y <= 0:  # Check if in the fourth quadrant
        #     self.theta1 = degrees(phi_2 + phi_1)   # Opposite angle for theta1
        #     self.theta2 = - self.theta2  # Opposite angle for theta2 

        # if x == 200 and y == 200:
        #     self.theta1 = degrees(phi_2 + phi_1)   # Opposite angle for theta1
        #     self.theta2 = - self.theta2  # Opposite angle for theta2 


        return self.theta1,self.theta2
    

    # Homogeneous transformation generator: (Base2Gripper):
    def Base2Grip(self):
        theta1_rad = deg2rad(self.theta1)# * pi / 180
        theta2_rad = deg2rad(self.theta2)# * pi / 180

        T0_1 = [ # transformation from base2joint1
            [cos(theta1_rad), -sin(theta1_rad), 0, L1 * cos(theta1_rad)],
            [sin(theta1_rad), cos(theta1_rad),  0, L1 * sin(theta1_rad)],
            [0,                   0,                    1, 0],
            [0,                   0,                    0, 1]
        ]

        T1_2 = [ # transformation from joint12Gripper
            [cos(theta2_rad), -sin(theta2_rad), 0, L2 * cos(theta2_rad)],
            [sin(theta2_rad), cos(theta2_rad),  0, L2 * sin(theta2_rad)],
            [0,                   0,                    1, 0],
            [0,                   0,                    0, 1]
        ]

        TB_G = dot(T0_1,T1_2)    # transformation from base to gripper 
        TB_G[2,3] = self.z       # adding z value to transformation from base2gripper
        # savez('pose{}.npz'.format(i), TB_G=TB_G) # example
        return TB_G


    # Serial write data
    def updateData(self):
        self.data = str(self.saveStatus)+","+str(self.flip)+","+str(self.j1Slider)+","+str(self.j2Slider)+","+str(self.j3Slider)+","+str(self.zSlider)+","+str(self.servoValue)+","+str(self.speedSlider)+","+str(self.accelerationSlider)
        # Data:
            # data[0] -> dummy1
            # data[1] -> dummy2
            # data[2] -> Joint 1 angle
            # data[3] -> Joint 2 angle
            # data[4] -> Joint 3 angle
            # data[5] -> Z position (in mm)
            # data[6] -> Servo value
            # data[7] -> Speed value
            # data[8] -> Acceleration value
        #return self.data 


    # Move to (x,y) coordinate
    def moveTo(self,x,y):
        try:
            T1,T2 = self.inverseKinematics(x,y)
            self.j1Slider = round(T1)
            self.j2Slider = round(T2)
            # return self.updateData()
            self.updateData()  # stores updated movement in self.data variable
            self.write_wait()  # write and wait's till confirmation
            self.skip = False
        except:
            self.skip = True
            print("Error in IK -> Position of object is beyond robot's reach")

    # ********* WIP *********
    def zMoveTo(self,dist): # dist in mm  # 500 steps per mm
        self.speedSlider = 300
        self.zSlider = dist   # in mm 
        self.updateData()
        self.write_wait()
        self.speedSlider = 500


    # Servo control (for gripper):
    def writeServo(self,angle):
        self.servoValue = angle
        self.updateData()
        self.write_wait()

    # Exit Connection:
    def arduStop(self):
        self.ser_port.close()
        print("Serial Connection Stopped")

    # Camera frame to Robot frame conversion:
    def Camframe2Robotframe(self,TC_O):
        # TC_O is a 4x4 matrix
        file = load("./TC_B.npz")
        TC_B = np_round(file['arr_0'])    # TC_B -> transformation from camera to base (Hand-Eye calibration)

        # for now i consider TC_B == TB_C
        TB_C = TC_B #linalg.inv(TC_B)

        # homogeneous transformation from base to object(point) in image
        TB_O = dot(TB_C,TC_O)
        # print(TB_O)
        # seperate the point (x,y,z) coordinate in robot frame
        (x,y,z) = TB_O[0:3,3]

        return (x,y,z)

