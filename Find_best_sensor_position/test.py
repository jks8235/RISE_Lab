#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
import math

data = np.load('/media/jee/FC12-B7D8/data_folder/Best_sensor_position/20_resol.npy')

print(data, data.shape)

print(data.shape[0])
sensor_pose_data = data[:,:,0:7]
sensor_collision_data = data[:,:,7:11]

# print(sensor_pose_data, sensor_pose_data.shape)
print(sensor_collision_data, sensor_collision_data.shape)

# print(sensor_pose_data[0,0,:])

# resolution = 15

# theta_0_range = range(-175, 175+1 ,resolution) # -175 ~ 175
# theta_1_range = range(-100, 100+1 ,resolution) # -175 ~ 175 
# theta_2_range = range(-150, 150+1, resolution) # -175 ~ 175
# theta_3_range = range(-175, 175+1 ,resolution) # -175 ~ 175
# theta_4_range = range(-175, 175+1 ,resolution) # -175 ~ 175
# # theta_5_range = range(-215, 215+1 ,resolution) # -215 ~ 215

# trajectory = []

# for theta_0 in theta_0_range:
#     for theth_1 in theta_1_range:
#         for theth_2 in theta_2_range:
#             for theth_3 in theta_3_range:
#                 for theth_4 in theta_4_range:
#                     trajectory += [[float(theta_0), float(theth_1), float(theth_2), float(theth_3), float(theth_4), 0.0]]

# print(len(trajectory))

# resolution = 0.001

# thresh_x = 0.005 # 센서 너비 range
# thresh_y = 0.003 # 센서 높이 range
# thresh_z = 0.01  # 센서 측정 거리

# point = []

# z_range = [resolution*i for i in range(int(thresh_z/resolution)+1)]

# for z in z_range:
#     x_max = thresh_x * math.sqrt(1-(z/thresh_z)**2)
#     x_range = [resolution*i for i in range(-int(x_max/resolution),int(x_max/resolution)+1)]
#     for x in x_range:
#         y_max = thresh_y * math.sqrt(1.000000001 - (x/thresh_x)**2 - (z/thresh_z)**2)
#         y_range = [resolution*i for i in range(-int(y_max/resolution),int(y_max/resolution)+1)]
#         for y in y_range:
#             point += [[x, y, z]]

# print(point)