#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np

def save_as_npy(data, name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/sonar/PAP_test_1/' + name + '.npy'
    np.save(path, data)

def load_from_npy(name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/sonar/PAP_test_1/' + name + '.npy'
    data = np.load(path)

    return data

if __name__ == "__main__":
    sensor_input = [0 for _ in range(5)]
    sensor_output = [0 for _ in range(5)]
    for fold_num in range(5):
        temp_sensor_input = [0 for _ in range(4)]
        temp_sensor_output = [0 for _ in range(4)]

        for sensor_num in range(4):
            temp_sensor_input[sensor_num] = load_from_npy('sensor_%d_input_Fold_%d'%(sensor_num+1, fold_num+1))
            temp_sensor_output[sensor_num] = load_from_npy('sensor_%d_output_Fold_%d'%(sensor_num+1, fold_num+1))
        
        sensor_input[fold_num] = np.concatenate((temp_sensor_input))
        sensor_output[fold_num] = np.concatenate((temp_sensor_output))

    total_input = np.concatenate((sensor_input))
    total_output = np.concatenate((sensor_output))

    print(total_input.shape, total_output.shape)

    save_as_npy(total_input, 'total_input')
    save_as_npy(total_output, 'total_output')

    # save_as_npy(sensor_1_input_np_data, 'sensor_1_input_Fold_%d'%(save_count))
    # save_as_npy(sensor_2_input_np_data, 'sensor_2_input_Fold_%d'%(save_count))
    # save_as_npy(sensor_3_input_np_data, 'sensor_3_input_Fold_%d'%(save_count))
    # save_as_npy(sensor_4_input_np_data, 'sensor_4_input_Fold_%d'%(save_count))
    # save_as_npy(sensor_1_output_np_data, 'sensor_1_output_Fold_%d'%(save_count))
    # save_as_npy(sensor_2_output_np_data, 'sensor_2_output_Fold_%d'%(save_count))
    # save_as_npy(sensor_3_output_np_data, 'sensor_3_output_Fold_%d'%(save_count))
    # save_as_npy(sensor_4_output_np_data, 'sensor_4_output_Fold_%d'%(save_count))