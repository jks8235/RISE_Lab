#!/usr/bin/env python
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import numpy as np
import rospy
import struct
import math
import time

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

### Base Calss set : publish point, publish point & rgb
class PointCloud_Sensor:
    def __init__(self):
        self.result = 0
        self.header_id = 'ground'
        self.Pub_name = 'sensor'
        self.Point_xyz = []

    def pcl_pub_xyzrgb(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.header_id

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
        ]

        pcl = point_cloud2.create_cloud(header, fields, points)
        pointcloud_publisher = rospy.Publisher(self.Pub_name, PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)
        
    def pcl_pub_xyz(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.header_id

        pcl = point_cloud2.create_cloud_xyz32(header, points)
        pointcloud_publisher = rospy.Publisher(self.Pub_name, PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)

        print pcl
        time.sleep(0.5)

    def shape_cone(self, distance):
        angle = 60
        angle_resolution = 5
        angle_iter = 1 + int((angle/2)/angle_resolution)

        for n in range(0, angle_iter):
            temp_angle = angle_resolution * n

            xy_radius = distance * math.sin(np.deg2rad(temp_angle))
            z = distance * math.cos(np.deg2rad(temp_angle))

            for m in range(0, int(360/angle_resolution) + 1):

                rotation_angle = m * angle_resolution

                x = xy_radius * math.cos(np.deg2rad(rotation_angle))
                y = xy_radius * math.sin(np.deg2rad(rotation_angle))

                Pt=[x, y, z]
                self.Point_xyz.append(Pt)
        time.sleep(0.5)
        self.pcl_pub_xyz(self.Point_xyz)