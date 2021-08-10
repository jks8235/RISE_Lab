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

    def pcl_pub_xyzrgb(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
        ]

        pcl = point_cloud2.create_cloud(header, fields, points)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)
        
    def pcl_pub_xyz(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl = point_cloud2.create_cloud_xyz32(header, points)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)
        print("Point Cloud published")

    def make_point(self, mode='test', thresh_hold=0.5):

        sensor_num = 17
        points = None
        pose = None

        if mode == 'test':
            for i in range(3):
                path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/ver1/test_4/output_Fold_%d.csv'%(i+1)
                path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/ver1/test_4/pose_Fold_%d.csv'%(i+1)
                temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
                temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

                points = merge_data(points, temp_points, axis=1)
                pose = merge_data(pose, temp_pose, axis=1)


        elif mode == 'predict':
            for i in range(3):
                path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/ver1/test_4/pose_Fold_%d.csv'%(i+1)
                temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

                pose = merge_data(pose, temp_pose, axis=1)

            path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/ver1/result/predict_Fold_5_4.csv'
            points = np.array(pd.read_csv(path1, sep=",", header=None))

        print('Data Load Done')
        print(points.shape)
        print(pose.shape)

        sensor_to_point_cloud = []

        for point_num in range(points.shape[1]):
            for sensor in range(sensor_num):

                if points[sensor+17, point_num] > thresh_hold:

                    Full_quarternion = pose[sensor*7:(sensor+1)*7, point_num]
                    point = [0.0, 0.0, points[sensor, point_num]]

                    translation = Full_quarternion[:3]
                    quarternion = Full_quarternion[3:].tolist()

                    rot_matrix = R.as_dcm(R.from_quat(quarternion))

                    world_point = (np.dot(rot_matrix, point) + translation).tolist()

                    sensor_to_point_cloud.append(world_point)

        self.pcl_pub_xyz(sensor_to_point_cloud)


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

    while not rospy.is_shutdown():
        try:
            point_cloud_vrep.make_point(mode='predict', thresh_hold=0.7)

        except rospy.ROSInterruptException:
            pass
