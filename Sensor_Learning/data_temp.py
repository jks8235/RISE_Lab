#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np

def load_data():
    total_input = []
    total_output = []

    for sensor_num in range(1,5):
        for fold_num in range(1,7):
            path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/sonar/PAP_test_1/sensor_%d_input_Fold_%d.npy'%(sensor_num, fold_num)
            path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/sonar/PAP_test_1/sensor_%d_output_Fold_%d.npy'%(sensor_num, fold_num)

            temp_input = np.load(path1)
            temp_output = np.load(path2)

            print(temp_input.shape, temp_output.shape)

            total_input += temp_input.tolist()
            total_output += temp_output.tolist()

    total_np_input = np.array(total_input)
    total_np_output = np.array(total_output)

    return total_np_input, total_np_output

def save_as_npy(data, name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/sonar/PAP_test_1/' + name + '.npy'
    np.save(path, data)


if __name__ == '__main__':

    total_np_input, total_np_output = load_data()

    print(total_np_input.shape, total_np_output.shape)

    save_as_npy(total_np_input, 'total_input')
    save_as_npy(total_np_output, 'total_output')