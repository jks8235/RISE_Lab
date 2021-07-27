#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
import rospy
import struct
import math
import time
import pandas as pd

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

    def make_point(self):
        path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/output_Fold_1.csv'
        path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/matrix_Fold_1.csv'

        points = np.array(pd.read_csv(path1, sep=",", header=None))
        matrixes = np.array(pd.read_csv(path2, sep=",", header=None))

        sensor_num = 17
        matrix_value_num = 12
        world_points = []

        # print(points.shape)
        # print(matrixes.shape)

        matrixes = matrixes.reshape(12,17,-1)

        # print(points.shape[1])
        # print(matrixes.shape)
        # print(len(matrixes[:,0,0]))

        for i in range(points.shape[1]):
            for j in range(sensor_num):
                point_list = [0, 0, points[j,i], 1]
                matrix_list = [matrixes[:,j,i]]

                point = np.array(point_list).reshape(4,1)
                matrix = np.array(matrix_list).reshape(3,4)

                world_point = np.dot(matrix,point)
                world_point_list = [world_point[0,0], world_point[1,0], world_point[2,0]]
                
                world_points.append(world_point_list)

        self.pcl_pub_xyz(world_points)

    def make_point_chek(self):
        path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/output_Fold_1.csv'
        path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/matrix_Fold_1.csv'

        points = np.array(pd.read_csv(path1, sep=",", header=None))
        matrixes = np.array(pd.read_csv(path2, sep=",", header=None))

        sensor_num = 17
        matrix_value_num = 12
        world_points = []

        # print(points.shape)
        # print(matrixes.shape)

        matrixes = matrixes.reshape(12,17,-1)

        # print(points.shape[1])
        # print(matrixes.shape)
        # print(len(matrixes[:,0,0]))

        for i in range(points.shape[1]):
            for j in range(sensor_num):
                point_list = [0, 0, points[j,i]]
                matrix_list = [matrixes[:,j,i]]

                point = np.array(point_list).reshape(3,1)
                tran_matrix = np.array(matrix_list).reshape(3,4)

                rot_mat = tran_matrix[:, 0:3]
                # print(rot_mat)
                inv_rot_mat = rot_mat.T
                trans_mat = tran_matrix[:, 3]
                # print(trans_mat)

                world_point = np.dot(inv_rot_mat,point) - tran_matrix
                world_point_list = [world_point[0,0], world_point[1,0], world_point[2,0]]
                
                world_points.append(world_point_list)

        self.pcl_pub_xyz(world_points)

### Main Code ###
if __name__ == '__main__':
    rospy.init_node('make_point_cloud')
    print("start")

    point_cloud_vrep = PointCloud_Shape()

    while not rospy.is_shutdown():
        try:
            point_cloud_vrep.make_point_chek()

        except rospy.ROSInterruptException:
            pass
