#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
from rospy.client import init_node

from std_msgs.msg import Int16MultiArray

rospy.init_node('ToF_Sensor', anonymous=True)

sensor_ros_msg = Int16MultiArray()

Sensor_data_publisher = rospy.Publisher('ToF_Sensor_data',
                                        Int16MultiArray,
                                        queue_size=2)

ser = serial.Serial(
    port='/dev/ttyS7',
    baudrate=9600,
    timeout=1
)# open serial port
print(ser.name) # check which port was really used
# s = ser.read(10)

while ser.is_open:
    if ser.readable():
        res = ser.readline()
        data_uni = res.decode()[:len(res)-1]
        data_uni_split = data_uni.split(',')
        del data_uni_split[-1]

        data_list = [int(x) for x in data_uni_split]
        
        sensor_ros_msg.data = data_list
        Sensor_data_publisher.publish(sensor_ros_msg)
