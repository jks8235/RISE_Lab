#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


class maker_array(object):
    def __init__(self):
        rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, self._maker_array_cb)#this obtain each joint pos

    def _maker_array_cb(self, data):
        count = 1
        # for maker in data.markers:
        #     # print([maker.pose.position.x, maker.pose.position.y, maker.pose.position.z])
        #     print(maker.scale)
        #     count += 1
        print('-----------------------------')

        print(len(data.markers[16].points))
        print(len(data.markers[16].colors))
        # array_points = np.array(data.markers[16].points)
        # array_colors = np.array(data.markers[16].colors)

        # print(array_points, array_colors)
        total_points = []
        for each_point in data.markers[16].points:
            total_points += [[each_point.x, each_point.y, each_point.z]]

        np_total_points = np.array(total_points)
        
        print(len(total_points), total_points[0])
        print(np_total_points.shape)

        np.save('/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/map/test_1_0.05_0.8.npy', np_total_points)
        # np.save('/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/sonar/map/test_1_0.05_0.8.npy', np_total_points)

        # for maker in data.markers:
        #     # print(maker.color)
        #     # print(maker.action)
        #     print(maker)
            # print("*****************************", count)
            # count += 1
        # print(data)

        print('-----------call end----------')

if __name__ == '__main__':
    rospy.init_node('maker_read')
    print("start_node")
    
    maker = maker_array()
    while not rospy.is_shutdown():
        pass
