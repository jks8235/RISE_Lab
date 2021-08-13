#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
import rospy
import struct
import math
import time
import pandas as pd
from scipy.spatial.transform import Rotation as R

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

### Base Calss set : publish point, publish point & rgb
class PointCloud_Shape:
    def __init__(self):
        self.result = 0

    def pcl_pub_xyz(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl = point_cloud2.create_cloud_xyz32(header, self.sensor_to_point_cloud)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)
        print("Point Cloud published")

    def load_data(self, mode='test'):
        self.points = None
        self.pose = None

        if mode == 'test':
            for i in range(1):
                path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/hexagon/test_1/output_Fold_%d.csv'%(i+1)
                path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/hexagon/test_1/pose_Fold_%d.csv'%(i+1)
                temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
                temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

                self.points = merge_data(self.points, temp_points, axis=1).T
                self.pose = merge_data(self.pose, temp_pose, axis=1).T


        elif mode == 'predict':
            for i in range(1):
                path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/hexagon/test_1/pose_Fold_%d.csv'%(i+1)
                temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

                self.pose = merge_data(self.pose, temp_pose, axis=1)

            path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/ver1/result/predict_Fold_5_4.csv'
            self.points = np.array(pd.read_csv(path1, sep=",", header=None)).T

        print('Data Load Done')
        print(self.points.shape)
        print(self.pose.shape)

    def make_point(self, thresh_hold=0.5):

        sensor_num = 19
        self.sensor_to_point_cloud = []

        for point_num in range(self.points.shape[1]):
            for sensor in range(sensor_num):

                if self.points[sensor+sensor_num, point_num] > thresh_hold:

                    Full_quarternion_1 = self.pose[sensor*7:(sensor+1)*7, point_num]
                    point_1 = [0.0, 0.0, self.points[sensor, point_num]]

                    translation_1 = Full_quarternion_1[:3]
                    quarternion_1 = Full_quarternion_1[3:].tolist()

                    rot_matrix_1 = R.as_dcm(R.from_quat(quarternion_1))

                    world_point_1 = (np.dot(rot_matrix_1, point_1) + translation_1).tolist()

                    self.sensor_to_point_cloud.append(world_point_1)

                sensor2 = sensor+19

                if self.points[sensor2+sensor_num, point_num] > thresh_hold:

                    Full_quarternion_2 = self.pose[sensor2*7:(sensor2+1)*7, point_num]
                    point_2 = [0.0, 0.0, self.points[sensor2, point_num]]

                    translation_2 = Full_quarternion_2[:3]
                    quarternion_2 = Full_quarternion_2[3:].tolist()

                    rot_matrix_2 = R.as_dcm(R.from_quat(quarternion_2))

                    world_point_2 = (np.dot(rot_matrix_2, point_2) + translation_2).tolist()

                    self.sensor_to_point_cloud.append(world_point_2)

            print("Making", point_num, self.points.shape[1])
        
        print("Making Done")

def merge_data(total_data, add_data, axis=0):
    total = total_data

    if type(total)==type(None):
        total = add_data
#         print('merge ok')
    else:
        total = np.concatenate((total, add_data),axis)
    
    total = pd.DataFrame(total)
    total = np.array(total)

    return total


### Main Code ###
if __name__ == '__main__':
    rospy.init_node('make_point_cloud')
    print("start")

    point_cloud_vrep = PointCloud_Shape()
    point_cloud_vrep.load_data(mode='test')
    point_cloud_vrep.make_point(thresh_hold=0.7)

    while not rospy.is_shutdown():
        try:
            point_cloud_vrep.pcl_pub_xyz()

        except rospy.ROSInterruptException:
            pass
