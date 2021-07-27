#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

## ToF Sensor TF
def sensor_TOF_TF(sensor_num=10):
    sensor_num = sensor_num # or 20
    theta_range = [2*math.pi/sensor_num*i for i in range(sensor_num)]

    for theta in theta_range:

        Trnasformation_bundle_bundle2 = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                                                  [math.sin(theta), math.cos(theta), 0, 0],
                                                  [0, 0, 1, 0],
                                                  [0, 0, 0, 1]])

        Trnasformation_bundle2_sensor = np.array([[1, 0, 0, 0],
                                                  [0, 0, 1, 90.579],
                                                  [0, -1, 0, -14.165],
                                                  [0, 0, 0, 1]])

        Trnasformation_bundle_sensor = np.dot(Trnasformation_bundle_bundle2, Trnasformation_bundle2_sensor)

        # print(Transformation_to_quaternion(Trnasformation_bundle_sensor))

## 4ch Sensor TF
def sensor_4ch_TF():
    sensor_num = 4
    theta_range = [2*math.pi/sensor_num*i for i in range(sensor_num)]

    for theta in theta_range:
        Trnasformation_end_bundle = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0],
                                              [0, 0, 1, 41.25],
                                              [0, 0, 0, 1]])

        Trnasformation_bundle_bundle2 = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                                                  [math.sin(theta), math.cos(theta), 0, 0],
                                                  [0, 0, 1, 0],
                                                  [0, 0, 0, 1]])

        Trnasformation_bundle2_sensor = np.array([[1, 0, 0, 0],
                                                  [0, 0, 1, 60],
                                                  [0, -1, 0, 0],
                                                  [0, 0, 0, 1]])

        Transformation_to_quaternion(Trnasformation_bundle2_sensor)

        Trnasformation_end_sensor = np.dot(np.dot(Trnasformation_end_bundle, Trnasformation_bundle_bundle2), Trnasformation_bundle2_sensor)

        # print(Transformation_to_quaternion(Trnasformation_end_sensor))


def Transformation_to_quaternion(Transformation_Matrix):
    Transformation_Matrix = Transformation_Matrix
    
    if Transformation_Matrix.shape != (4,4):
        print("check Transformation Matrix")
        return 0
    
    Rot = Transformation_Matrix[0:3, 0:3]
    Pos = Transformation_Matrix[0:3, 3].tolist()

    Quat = R.as_quat(R.from_dcm(Rot)).tolist()

    Total_Quat = Pos + Quat

    return Total_Quat


if __name__ == '__main__':
    sensor_4ch_TF()
    sensor_TOF_TF()