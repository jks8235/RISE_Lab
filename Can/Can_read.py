#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
from rospy.client import init_node

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

rospy.init_node('4ch_sensor', anonymous=True)

sensor_ros_msg = Float32MultiArray()

Sensor_data_publisher = rospy.Publisher('4ch_sensor',
                                        Float32MultiArray,
                                        queue_size=2)

ser = serial.Serial(
    port='/dev/ttyS5',
    baudrate=9600,
    timeout=1
)# open serial port
print(ser.name) # check which port was really used

while ser.is_open:
    if ser.readable():
        res = ser.readline()
        data_uni = res
        # data_uni = res.decode()[:len(res)-1]
        data_uni_split = data_uni.split(',')
        del data_uni_split[-1]

        data_8_HEX = [str(x) for x in data_uni_split]
        distance_data = []
        
        if (len(data_8_HEX) == 8):
            data_4_HEX = [data_8_HEX[0]+data_8_HEX[1], data_8_HEX[2]+data_8_HEX[3], data_8_HEX[4]+data_8_HEX[5], data_8_HEX[6]+data_8_HEX[7]]
            data_4_int = [int(x, 16) for x in data_4_HEX]
            
            distance_data = [float(x)/100 for x in data_4_int]
            print(distance_data)
        
        sensor_ros_msg.data = distance_data
        Sensor_data_publisher.publish(sensor_ros_msg)
