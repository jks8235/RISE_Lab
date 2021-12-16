#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np

map_target = np.round(np.load('/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/map/test_1_0.05_0.8.npy'), 3).tolist()
map_sonar = np.round(np.load('/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/sonar/map/test_1_0.05_0.8.npy'), 3).tolist()
map_predict = np.round(np.load('/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/map/test_1_predict_0.05_0.8.npy'), 3).tolist()

# print(map_target.shape, map_predict.shape, map_sonar.shape)

def compare(basic_map, compare_map):
    # type(map) : list
    comb_map = basic_map
    inter_map = []

    for point in compare_map:
        if point in comb_map:
            inter_map.append(point)
        else:
            comb_map.append(point)

    map_eval = float(len(inter_map))/float(len(comb_map))

    return map_eval

if __name__ == '__main__':
    target_sonar = compare(map_target, map_sonar)
    target_predict = compare(map_target, map_predict)
    sonar_predict = compare(map_sonar, map_predict)

    print(target_sonar, target_predict, sonar_predict)