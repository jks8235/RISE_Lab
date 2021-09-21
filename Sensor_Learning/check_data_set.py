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
        pointcloud_publisher = rospy.Publisher('octomap_point_cloud_centers', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)
        print("Point Cloud published")

    def load_data(self, mode='test'):
        self.points = None
        self.pose = None

        if mode == 'test':
            for i in range(1):
                path1 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/test_3/output_Fold_%d.csv'%(i+1)
                path2 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/test_3/pose_Fold_%d.csv'%(i+1)
                temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
                temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

                self.points = merge_data(self.points, temp_points, axis=1)
                self.pose = merge_data(self.pose, temp_pose, axis=1)

            self.points = self.points.T


        elif mode == 'predict':
            for i in range(1):
                path2 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/test_3/pose_Fold_%d.csv'%(i+1)
                temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

                self.pose = merge_data(self.pose, temp_pose, axis=1)

            path1 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/result/predict_Fold_1_3.csv'
            self.points = np.array(pd.read_csv(path1, sep=",", header=None))

        self.pose = self.pose.T

        print('Data Load Done')
        print(self.points.shape)
        print(self.pose.shape)

    def make_point(self, thresh_hold=0.5):

        sensor_num = 19
        self.sensor_to_point_cloud = []

        for point_num in range(self.points.shape[1]):
            for sensor in range(sensor_num):

                Full_quarternion_1 = self.pose[sensor*7:(sensor+1)*7, point_num]

                translation_1 = Full_quarternion_1[:3]

                self.sensor_to_point_cloud.append(translation_1)

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
    point_cloud_vrep.load_data(mode='predict')
    point_cloud_vrep.make_point(thresh_hold=0.7)

    while not rospy.is_shutdown():
        try:
            point_cloud_vrep.pcl_pub_xyz()

        except rospy.ROSInterruptException:
            pass
