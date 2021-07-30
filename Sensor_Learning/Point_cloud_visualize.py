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

    def make_point(self):

        sensor_num = 17

        path1 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/output_Fold_1.csv'
        path2 = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/pose_Fold_1.csv'

        points = np.array(pd.read_csv(path1, sep=",", header=None))
        # points = points[:sensor_num,:]
        pose = np.array(pd.read_csv(path2, sep=",", header=None))

        print(points.shape)
        print(pose.shape)

        sensor_to_point_cloud = []

        for point_num in range(points.shape[1]):
            for sensor in range(sensor_num):

                Full_quarternion = pose[sensor*7:(sensor+1)*7, point_num]
                point = [0.0, 0.0, points[sensor, point_num]]

                translation = Full_quarternion[:3]
                quarternion = Full_quarternion[3:].tolist()

                print(type(translation))
                print(translation)

                print(type(quarternion))
                print(quarternion)

                rot_matrix = R.as_dcm(R.from_quat(quarternion))

                # print(type(rot_matrix))
                # print(rot_matrix)

                # print(type(point))
                # print(point)

                world_point = (np.dot(rot_matrix, point) + translation).tolist()

                print(type(world_point))
                print(world_point)

                sensor_to_point_cloud.append(world_point)

        self.pcl_pub_xyz(sensor_to_point_cloud)

### Main Code ###
if __name__ == '__main__':
    rospy.init_node('make_point_cloud')
    print("start")

    point_cloud_vrep = PointCloud_Shape()

    while not rospy.is_shutdown():
        try:
            point_cloud_vrep.make_point()

        except rospy.ROSInterruptException:
            pass
