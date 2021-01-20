#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import rospy

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header

###
import tf
import time

class SensorData:
    def __init__(self,r,theta,sensor_obj):
        #default 0.3[m]
        self.r=r
        self.theta=theta
        self.distance_resolution=0.000001
        self.angle_resolution=5.0
        self.angle_range=1+(int(self.theta/self.angle_resolution))/2
        self.sensor_obj = sensor_obj
        # self.br = tf.TransformBroadcaster()
        # self.br.sendTransform([0.0,0.0,0.0],
        #                     self.e2q([0.0,0.0,0.0]),
        #                     rospy.Time.now(),
        #                     'test_pcl',
        #                     'map'
        #                     )

    def e2q(self,value):
        return tf.transformations.quaternion_from_euler(value[0],value[1],value[2])

    def SensorDataCB(self,data):
        angle=0.0
        prev_array=[]
        for test_step in range(3):
            if  self.r > data and data > 0.001:
                for n in range(0,int(self.theta/self.angle_resolution/2)+1):
                    angle=self.angle_resolution*n
                    
                    s = math.sin(np.deg2rad(float(angle)))
                    c = math.cos(np.deg2rad(float(angle)))
                    # t = math.tan(np.deg2rad(angle))
                    z=data*c
                    x=data*s
                    y=data*s
                    for r in range(0,int(360/self.angle_resolution)+1):
                        rot_x=x*math.cos(np.deg2rad(r*self.angle_resolution))+y*math.sin(np.deg2rad(r*self.angle_resolution))
                        rot_y=-x*math.sin(np.deg2rad(r*self.angle_resolution))+y*math.cos(np.deg2rad(r*self.angle_resolution))
                        if z > 0:
                            pcl_g=[rot_x + test_step*0.5, rot_y, z]
                            prev_array.append(pcl_g)
            time.sleep(0.5)
            self.pcl_pub(prev_array)

    def pcl_pub(self,prev_array):
        header = Header()
        header.stamp = rospy.Time.now()
        # header.frame_id = '{}'.format(self.sensor_obj)
        header.frame_id = 'map'
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, prev_array)
        # pointcloud_publisher = rospy.Publisher("{}".format(self.sensor_obj), PointCloud2, queue_size=10)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(scaled_polygon_pcl)

if __name__ == "__main__":
    rospy.init_node('make_point_cloud')
    SD = SensorData(0.151, 120.0, "test_pcl")
    r= rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            SD.SensorDataCB(0.15)
        except rospy.ROSInterruptException:
            pass
        r.sleep()