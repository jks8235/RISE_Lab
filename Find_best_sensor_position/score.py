#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Dr.Kyungsub Jee (King of the RISE) 양아치

import numpy as np
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
        self.br = tf.TransformBroadcaster()
        self.header = Header()
        self.pointcloud_publisher = rospy.Publisher('test/pcl', PointCloud2, queue_size=10)
        
        self.sensor_pose_data = data[:,:,0:7]
        self.sensor_collision_data = data[:,:,7:11]

        self.bundle_transformation_pub = rospy.Publisher(
            'bundle_transformation_data',
            Float32MultiArray,
            queue_size=2
        )

        self.sensor_mounts = [0 for i in range(17)]

        self.sensor_tf = [
            [0, -0.02, 0, 0.7071069478988647, 0, 0, 0.7071067094802856],
            [0.02, 0, 0, 0.5, 0.5, 0.5, 0.5],
            [0, 0.02, 0, 0, 0.7071068286895752, 0.7071068286895752, 0],
            [-0.02, 0, 0, 0.5, -0.5, -0.5, 0.5]
        ]

    def get_point_cloud(self, points):
        header = self.header
        header.stamp = rospy.Time.now()
        header.frame_id = "sensor"

        fields = [PointField('x', 0, PointField.FLOAT32,1),
                  PointField('y', 4, PointField.FLOAT32,1),
                  PointField('z', 8, PointField.FLOAT32,1),
                  PointField('intensity', 12, PointField.FLOAT32,1)]

        pcl = point_cloud2.create_cloud(header, fields, points)
        return pcl

    def get_tf_point_cloud_data(self, full_quarternion_bundle, point_cloud_data):

        pcl_transform = geometry_msgs.msg.TransformStamped()
        sensor_to_point_cloud = []
        
        point_cloud = self.get_point_cloud(point_cloud_data)

        pcl_transform.header = self.header
        pcl_transform.header.stamp = rospy.Time.now()
        pcl_transform.header.frame_id = "sensor_bundle"
        pcl_transform.child_frame_id = "sensor"
        pcl_transform.transform.translation.x = full_quarternion_bundle[0]
        pcl_transform.transform.translation.y = full_quarternion_bundle[1]
        pcl_transform.transform.translation.z = full_quarternion_bundle[2]
        pcl_transform.transform.rotation.x = full_quarternion_bundle[3]
        pcl_transform.transform.rotation.y = full_quarternion_bundle[4]
        pcl_transform.transform.rotation.z = full_quarternion_bundle[5]
        pcl_transform.transform.rotation.w = full_quarternion_bundle[6]
        
        '''
        do_transform_cloud
        @First param: point cloud data
        @Second param: The Transform message to convert.
        @Second param type: geometry_msgs.msg.TransformStamped
        '''
        cloud_out = do_transform_cloud(point_cloud, pcl_transform)
        sensor_to_point_cloud.append(cloud_out)

        return sensor_to_point_cloud

    def make_sensor_point_cloud(self, resolution = 0.001):
        thresh_x = 0.005 # 센서 너비 range
        thresh_y = 0.003 # 센서 높이 range
        thresh_z = 0.01  # 센서 측정 거리

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

    def make_point(self, sensor_mount_number):

        ellipse_point_cloud = self.make_sensor_point_cloud()

        pose = self.sensor_pose_data
        sensor_point_cloud = []
        total_point_cloud = []

        for point_num in range(self.sensor_pose_data.shape[0]):
            for sensor in range(4):
                
                each_sensor_point_cloud = []

                for point in range(len(ellipse_point_cloud)):
                    if self.sensor_collision_data[point_num, sensor_mount_number, sensor] != 0.:
                        print(self.sensor_collision_data[point_num, sensor_mount_number, sensor])
                    intensity = 0.1
                
                    full_quarternion_sensor = self.sensor_tf[sensor]
                    ellipse_point = ellipse_point_cloud[point]

                    translation = full_quarternion_sensor[:3]
                    quarternion = full_quarternion_sensor[3:]

                    rot_matrix = R.as_dcm(R.from_quat(quarternion))

                    tf_ellipse_point = (np.dot(rot_matrix, ellipse_point) + translation).tolist()
                    tf_ellipse_point.append(intensity)

                    each_sensor_point_cloud.append(tf_ellipse_point)

                sensor_point_cloud += each_sensor_point_cloud

            full_quarternion_bundle = self.sensor_pose_data[point_num, sensor_mount_number, :]
            self.bundle_transformation_msg = Float32MultiArray()
            self.bundle_transformation_msg.data = full_quarternion_bundle
            self.bundle_transformation_pub.publish(self.bundle_transformation_msg)

            pcl = self.get_tf_point_cloud_data(full_quarternion_bundle, sensor_point_cloud)

            if len(pcl) != 0:
                transform_pc = copy.copy(pcl[0])
                for pc in pcl[1:]:
                    transform_pc.width += pc.width
                    transform_pc.data += pc.data
    
            self.pointcloud_publisher.publish(transform_pc)
            time.sleep(0.001)

            ("Making", point_num, self.sensor_pose_data.shape[0])
        
        print("Making Done")

if __name__ == '__main__':

    rospy.init_node('make_point_cloud')
    print("start")

    data = np.load('/media/jee/FC12-B7D8/data_folder/Best_sensor_position/20_resol.npy')

    point_cloud_vrep = Score(data)

    count = 0

    while not rospy.is_shutdown():
        try:
            if count < 1:
                count += 1 
                point_cloud_vrep.make_point(0)

        except rospy.ROSInterruptException:
            pass