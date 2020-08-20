#!/usr/bin/env python
# encoding: utf-8

import rospy
import serial
from serial import Serial
import numpy as np
from numpy import pi
from sensor_msgs.msg import Imu

ACCData=[0.0]*8
GYROData=[0.0]*8
AngleData=[0.0]*8          
FrameState = 0     
Bytenum = 0           
CheckSum = 0                  

#acceleration
a = [0.0]*3
#angular velocity
w = [0.0]*3
#angle
Angle = [0.0]*3
#angle_in_quat
Angle_quat = [0.0]*4

def DueData(inputdata):   

    global  FrameState   
    global  Bytenum
    global  CheckSum
    global  a
    global  w
    global  Angle
    global  Angle_quat

    for data in inputdata:  
        data = ord(data)
        if FrameState==0:   
            if data==0x55 and Bytenum==0: 
                CheckSum=data
                Bytenum=1
                continue
            elif data==0x51 and Bytenum==1:
                CheckSum+=data
                FrameState=1
                Bytenum=2
            elif data==0x52 and Bytenum==1:
                CheckSum+=data
                FrameState=2
                Bytenum=2
            elif data==0x53 and Bytenum==1:
                CheckSum+=data
                FrameState=3
                Bytenum=2
        elif FrameState==1: # acc 
            
            if Bytenum<10:           
                ACCData[Bytenum-2]=data 
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff): 
                    a = get_acc(ACCData)
                CheckSum=0                 
                Bytenum=0
                FrameState=0
        elif FrameState==2: # gyro
            
            if Bytenum<10:
                GYROData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    w = get_gyro(GYROData)
                CheckSum=0
                Bytenum=0
                FrameState=0
        elif FrameState==3: # angle
            
            if Bytenum<10:
                AngleData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    Angle = get_angle(AngleData)
                    Angle_quat = euler_to_quaternion(Angle[0], Angle[1], Angle[2])

                    #change acceleration to m/s2 (by default the unit is g)
                    a = [9.81*i for i in a]
                    a = tuple(a)
 
                    print "acceleration(m/s2):\t%10.3f %10.3f %10.3f"%a
                    print "angular vel(deg/s):\t%10.3f %10.3f %10.3f"%w
                    print "angle(deg):\t\t%10.3f %10.3f %10.3f"%Angle
                    print "angle_quat:\t\t%10.3f %10.3f %10.3f %10.3f"%Angle_quat
                    print "====================================================================="
                    
                    publisher()

                CheckSum=0
                Bytenum=0
                FrameState=0
 
def publisher():

    global  a
    global  w
    global  Angle
    global  Angle_quat
    
    #publish imu
    imu_raw = Imu()
    imu_raw.header.stamp = rospy.Time.now()
    imu_raw.header.frame_id = "world"
    # imu_raw.header.seq = seq

    imu_raw.orientation.x = Angle_quat[0]
    imu_raw.orientation.y = Angle_quat[1]
    imu_raw.orientation.z = Angle_quat[2]
    imu_raw.orientation.w = Angle_quat[3]
    imu_raw.orientation_covariance[0] = 0

    imu_raw.linear_acceleration.x = a[0]
    imu_raw.linear_acceleration.y = a[1]
    imu_raw.linear_acceleration.z = a[2]
    imu_raw.linear_acceleration_covariance[0] = 0

    imu_raw.angular_velocity.x = w[0]
    imu_raw.angular_velocity.y = w[1]
    imu_raw.angular_velocity.z = w[2]
    imu_raw.angular_velocity_covariance[0] = 0

    pub_raw_imu.publish(imu_raw)


def get_acc(datahex):  

    axl = datahex[0]                                        
    axh = datahex[1]
    ayl = datahex[2]                                        
    ayh = datahex[3]
    azl = datahex[4]                                        
    azh = datahex[5]

    k_acc = 16.0

    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z-= 2 * k_acc

    return acc_x,acc_y,acc_z
 
 
def get_gyro(datahex):                                      
    wxl = datahex[0]                                        
    wxh = datahex[1]
    wyl = datahex[2]                                        
    wyh = datahex[3]
    wzl = datahex[4]                                        
    wzh = datahex[5]
    k_gyro = 2000.0
 
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >=k_gyro:
        gyro_z-= 2 * k_gyro
    return gyro_x,gyro_y,gyro_z
 
 
def get_angle(datahex):             

    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
 
    return angle_x,angle_y,angle_z


def euler_to_quaternion(roll, pitch, yaw):
    
    # change to radius
    yaw = yaw*pi/180
    pitch = pitch*pi/180
    roll = roll*pi/180

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return qx, qy, qz, qw
 

if __name__=='__main__': 

    rospy.init_node('jy901_imu', anonymous=True)
    pub_raw_imu = rospy.Publisher("raw_imu", Imu, queue_size=1)
    # use raw_input function for python 2.x or input function for python3.x
    # port = raw_input('please input port No. such as com7:');
    # #port = input('please input port No. such as com7:'));
    # baud = int(input('please input baudrate(115200 for JY61 or 9600 for JY901):'))
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate='9600', timeout=1)  # ser = serial.Serial('com7',115200, timeout=0.5) 
    print(ser.is_open)

    while not rospy.is_shutdown():
        datahex = ser.read(33)
        DueData(datahex) 
    
    print("Shutting down")
