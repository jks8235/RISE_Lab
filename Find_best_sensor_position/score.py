#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Dr.Kyungsub Jee (King of the RISE) 양아치

import numpy as np
from numpy.core.fromnumeric import shape
from numpy.lib.polynomial import RankWarning
from scipy.spatial import transform
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

        print("Mount_%d" %(sensor_mount_number+1))
        print("Sensor_1 Collision : %f" %(sensor_1_collision_ratio))
        print("Sensor_2 Collision : %f" %(sensor_2_collision_ratio))
        print("Sensor_3 Collision : %f" %(sensor_3_collision_ratio))
        print("Sensor_4 Collision : %f" %(sensor_4_collision_ratio))
        print("total Collision : %f" %(total_ratio))

    def detection_range_score(self, sensor_mount_number):
        sensor_mount_pose_data = self.sensor_pose_data[:,sensor_mount_number,:]
        sensor_mount_collision_data = self.sensor_collision_data[:,sensor_mount_number,:]

        senor_model_points = self._make_sensor_model_pcl()
        # print("sensor model done")

        data_num = self.sensor_pose_data.shape[0]

        # data_num = 2000

        total_pcl_list = ['0' for i in range(data_num)]

        for point_num in range(data_num):
            if sensor_mount_collision_data[point_num,:].tolist() == [1.0, 1.0, 1.0, 1.0]:
                pass
            else:
                sensor_mount_points = self._make_world_pcl(senor_model_points, sensor_mount_pose_data[point_num, :], sensor_mount_collision_data[point_num,:])
                total_pcl_list[point_num] = sensor_mount_points
            # print("%d/%d" %(point_num, data_num))

        while '0' in total_pcl_list:
            total_pcl_list.remove('0')

        total_pcl_array = np.concatenate(total_pcl_list, axis=0)
        # print(total_pcl_array.shape)
        total_pcl_array = np.unique(total_pcl_array, axis=0)
        print("Mount_%d" %(sensor_mount_number+1))
        print("Sensor_Range_vexel_num : %d" %(total_pcl_array.shape[0]))

        # self._pcl_pub_xyz(total_pcl)
        # time.sleep(0.01)

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

    def _make_world_pcl(self, sensor_model_points, mount_tf, sensor_check):

        total_points = []

        for sensor in range(4):
            if sensor_check[sensor] != 0.:
                pass
            else:
                full_quart_S2M = self.sensor_tf[sensor]
                full_quart_M2W = mount_tf

                transform_matrix_S2M = self._full_quart_to_transform_matrix(full_quart_S2M)
                transfrom_matrix_M2W = self._full_quart_to_transform_matrix(full_quart_M2W)

                transform_matrix_total = (np.dot(transfrom_matrix_M2W, transform_matrix_S2M))

                rot_matrix_total = transform_matrix_total[0:3,0:3]
                transl_matrix_total = transform_matrix_total[0:3, 3]

                points_array = np.array(sensor_model_points).T

                transformed_points = np.around((np.dot(rot_matrix_total, points_array).T + transl_matrix_total),2).tolist()
            
                total_points += transformed_points

        total_point_array = np.array(total_points)

        # print(total_point_array.shape)

        return total_point_array

    def _full_quart_to_transform_matrix(self, full_quart):
        translation = full_quart[:3]
        quarternion = full_quart[3:]

        rot_matrix = R.as_dcm(R.from_quat(quarternion))
        transl_matrix = np.array([translation]).T

        transform_matrix = np.append(np.append(rot_matrix, transl_matrix, axis=1), np.array([[0., 0., 0., 1.]]), axis=0)

        return transform_matrix

if __name__ == '__main__':

    data = np.load('/media/jee/FC12-B7D8/data_folder/Best_sensor_position/30_resol.npy')

    score = Score(data)

    score.detection_range_score(1)

    # for i in range(17):
    #     score.collision_score(i)
    #     print("")

    for i in range(17):
        score.detection_range_score(i)
        print("")

    # rospy.init_node('make_point_cloud')

    # score.detection_range_score(0)

    # print("start")

    # point_cloud_vrep = Score(data)

    # while not rospy.is_shutdown():
    #     try:
    #         score.detection_range_score(0)

    #     except rospy.ROSInterruptException:
    #         pass