#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
from scipy.spatial import distance

map_target = np.round(np.load('/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/map/220610/test_1_target_0.02_0.8.npy'), 3)

# map_dense = [np.round(np.load('/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/map/220619/test_1_dense_0.02_0.8(ratio_0.%d).npy'%(i)), 3) for i in range(11)]

map_dense =  [np.round(np.load('/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/map/test_1_dense_0.02_0.8(ratio_0.0955).npy'), 3)]

print(len(map_dense))
def closest_node(node, nodes):
    closest_index = distance.cdist([node], nodes).argmin()

    error = np.linalg.norm(node - nodes[closest_index])

    return error

if __name__ == '__main__':
    for compare_map in map_dense:
        error = []
        for point in range(compare_map.shape[0]):
            error += [closest_node(compare_map[point,:], map_target)]
        point_num = len(compare_map)
        error_sum = np.round(np.sum(error),5)
        error_mean = np.round(np.mean(error),5)
        error_var = np.round(np.var(error),5)

        print(point_num, error_sum, error_mean, error_var)
    # print('-------------------------')