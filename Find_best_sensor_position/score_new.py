#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Dr.Kyungsub Jee (King of the RISE) 양아치

import numpy as np
from numpy.core.fromnumeric import shape
import rospy
import struct
import math
import time
import pandas as pd
from scipy.spatial.transform import Rotation as R
import tf
import copy

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray

class Score:
    def __init__(self, data):
        self.sensor_pose_data = data[:,:,0:7]
        self.sensor_collision_data = data[:,:,7:11]

        self.sensor_tf = [
            [0, -0.021, 0, 0.7071069478988647, 0, 0, 0.7071067094802856],
            [0.021, 0, 0, 0.5, 0.5, 0.5, 0.5],
            [0, 0.021, 0, 0, 0.7071068286895752, 0.7071068286895752, 0],
            [-0.021, 0, 0, 0.5, -0.5, -0.5, 0.5]
        ]

    def collision_score(self, sensor_mount_number):
        sensor_mount_collision_data = self.sensor_collision_data[:,sensor_mount_number,:]

        sensor_1_collision_ratio = np.average(sensor_mount_collision_data[:,0])
        sensor_2_collision_ratio = np.average(sensor_mount_collision_data[:,1])
        sensor_3_collision_ratio = np.average(sensor_mount_collision_data[:,2])
        sensor_4_collision_ratio = np.average(sensor_mount_collision_data[:,3])
        total_ratio = np.average(sensor_mount_collision_data[:,0:4])

        print(sensor_1_collision_ratio, sensor_2_collision_ratio, sensor_3_collision_ratio, sensor_4_collision_ratio, total_ratio)

    def detection_range_score(self, sensor_mount_number):
        sensor_mount_pose_data = self.sensor_pose_data[:,sensor_mount_number,:]
        sensor_mount_collision_data = self.sensor_collision_data[:,sensor_mount_number,:]

        senor_model_points = self._make_sensor_model_pcl()
        print("sensor model done")

        total_pcl = []

        for point_num in range(self.sensor_pose_data.shape[0]):
            sensor_mount_points = self._make_sensor_mount_pcl(senor_model_points, sensor_mount_collision_data[point_num,:])
            mount_tf_points = self._make_mount_tf_pcl(sensor_mount_points, sensor_mount_pose_data[point_num, :])
            total_pcl.append(mount_tf_points)
            print("%d/%d" %(point_num, self.sensor_pose_data.shape[0]))

        self._pcl_pub_xyz(total_pcl)
        time.sleep(0.01)

    def _pcl_pub_xyz(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl = point_cloud2.create_cloud_xyz32(header, points)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)

    def _make_sensor_model_pcl(self, resolution = 0.01):
        thresh_x = 0.05 # 센서 너비 range
        thresh_y = 0.03 # 센서 높이 range
        thresh_z = 0.1  # 센서 측정 거리

        point = []

        z_range = [resolution*i for i in range(int(thresh_z/resolution)+1)]

        for z in z_range:
            x_max = thresh_x * math.sqrt(1-(z/thresh_z)**2)
            x_range = [resolution*i for i in range(-int(x_max/resolution),int(x_max/resolution)+1)]
            for x in x_range:
                y_max = thresh_y * math.sqrt(1.000000001 - (x/thresh_x)**2 - (z/thresh_z)**2)
                y_range = [resolution*i for i in range(-int(y_max/resolution),int(y_max/resolution)+1)]
                for y in y_range:
                    point += [[x, y, z]]

        return point

    def _make_sensor_mount_pcl(self, sensor_model_points, sensor_check):
        sensor_mount_pcl = []

        for sensor in range(4):
            if sensor_check[sensor] != 0.:
                pass
            else:
                for point in sensor_model_points:
                    full_quarternion_sensor = self.sensor_tf[sensor]

                    translation = full_quarternion_sensor[:3]
                    quarternion = full_quarternion_sensor[3:]

                    rot_matrix = R.as_dcm(R.from_quat(quarternion))

                    temp_point = (np.dot(rot_matrix, point) + translation).tolist()

                    sensor_mount_pcl.append(temp_point)

        return sensor_mount_pcl

    def _make_mount_tf_pcl(self, mount_points, mount_pose):
        mount_tf_pcl = []

        for point in mount_points:
            full_quarternion_sensor = mount_pose

            translation = full_quarternion_sensor[:3]
            quarternion = full_quarternion_sensor[3:]

            rot_matrix = R.as_dcm(R.from_quat(quarternion))

            temp_point = (np.dot(rot_matrix, point) + translation).tolist()

            mount_tf_pcl.append(temp_point)

        return mount_tf_pcl

if __name__ == '__main__':

    data = np.load('/media/jee/FC12-B7D8/data_folder/Best_sensor_position/20_resol.npy')

    score = Score(data)

    # score.collision_score(1)

    rospy.init_node('make_point_cloud')

    score.detection_range_score(0)

    # print("start")

    # point_cloud_vrep = Score(data)

    count = 0

    while not rospy.is_shutdown():
        try:
            score.detection_range_score(0)

        except rospy.ROSInterruptException:
            pass