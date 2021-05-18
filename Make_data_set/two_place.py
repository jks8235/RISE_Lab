#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
import math
import pandas as pd

class ToF_Sensor:
    def __init__(self, FOV=25, resolution=5):
        self.FOV = FOV
        self.resolution = resolution
        self.unit_angle = self.FOV/self.resolution

        self.FOV_distance = np.zeros([self.resolution, self.resolution])
        self.FOV_distance_angle = np.zeros([self.resolution, self.resolution])
        self.FOV_distance_multiplier = np.zeros(self.FOV_distance.shape)
        self.Position = [0, 0, 0]
        self.mid_point = int(math.ceil(self.resolution/2.0))

        for i in range(0, self.resolution):
            for j in range(0, self.resolution):
                self.FOV_distance_angle[i,j] = self.unit_angle*(math.sqrt((i+1-self.mid_point)**2 + (j+1-self.mid_point)**2))

        for i in range(0, self.resolution):
            for j in range(0, self.resolution):
                self.FOV_distance_multiplier[i,j] = 1/math.cos(self.deg2rad(self.FOV_distance_angle[i,j]))

        off_set = np.mean(self.FOV_distance_multiplier) - 1
        self.FOV_distance_multiplier = self.FOV_distance_multiplier - off_set*np.ones(self.FOV_distance_multiplier.shape)

    def set_map(self, distance):
        self.distance = distance

        self.Map_size = 880
        self.Map = np.zeros([self.Map_size, self.Map_size])
        self.Map[:int(self.Map_size/2),:] = self.distance[0]
        self.Map[:int(self.Map_size/2),:int(self.Map_size/2)] = self.distance[1]

        self.set_start()
        
    def get_distance(self):
        distnace = np.mean(self.FOV_distance)
        return distnace

    def get_FOV_distance(self, position):
        d = min(self.distance) - position[2]

        for i in range(0, self.resolution):
            for j in range(0, self.resolution):
                self.FOV_distance[i,j] = self.Map[position[0]+int(math.ceil(d*math.tan(self.deg2rad(self.unit_angle*(i+1-self.mid_point))))), position[1]+int(math.ceil(d*math.tan(self.deg2rad(self.unit_angle*(j+1-self.mid_point)))))]
        
        self.FOV_distance -= position[2]*np.ones(self.FOV_distance.shape)
        self.FOV_distance = self.FOV_distance*self.FOV_distance_multiplier
        return self.FOV_distance

    def deg2rad(self, deg):
        return deg/180.0*math.pi

    def set_start(self):
        r_1 = int(math.ceil(self.distance[0]*math.tan(self.deg2rad(self.FOV/2))))
        r_2 = int(math.ceil(self.distance[1]*math.tan(self.deg2rad(self.FOV/2))))
        r_max = max(r_1, r_2) + 30

        x_s = r_1
        y_s = self.Map.shape[1]/2
        z_s = 0

        self.Move_range = [[self.Map_size/2-r_max, self.Map_size/2+r_max], [self.Map_size/2-r_max, self.Map_size/2+r_max], [max(self.distance[0], self.distance[1])-850, max(self.distance[0], self.distance[1])-50]]
        self.Position = [x_s, y_s, z_s]

def save_data_as_csv(data, name):
    path = '/home/jee/catkin_ws/src/RISE_Lab/Make_data_set/data/' + name + '.csv'
    data.to_csv(path, sep=',', header=None, index=None)

def make_data(sensor, d_resolution, move_resolution):

    input_data = []
    verify_data = []
    output_data = []

    for d_1 in range(50, 851, d_resolution):
        for d_2 in range(50, 851,  d_resolution):
            d = [d_1, d_2]
            sensor.set_map(d)
    
            for x in range(sensor.Move_range[0][0], sensor.Move_range[0][1], move_resolution):
                for y in range(sensor.Move_range[1][0], sensor.Move_range[1][1], move_resolution):
                    FOV_1 = sensor.get_FOV_distance([x, y, 0])
                    distance_1 = sensor.get_distance()

                    move_distance = 20

                    for delta_x in range(-move_distance, move_distance+1, 10):
                        for delta_y in range(-(move_distance-abs(delta_x)), (move_distance-abs(delta_x))+1, 10):
                            for delta_z in range(-(move_distance-abs(delta_x)-abs(delta_y)), (move_distance-abs(delta_x)-abs(delta_y))+1, 10):
                                FOV_2 = sensor.get_FOV_distance([x+delta_x, y+delta_y, 0+delta_z])
                                distance_2 = sensor.get_distance()

                                temp_input = [delta_x, delta_y, delta_z, distance_1, distance_2]
                                temp_verify = []
                                temp_output = [distance_2]

                                for i in range(0, sensor.resolution):
                                    for j in range(0, sensor.resolution):
                                        temp_verify.append(FOV_2[j, sensor.resolution-i-1])

                                input_data.append(temp_input)
                                verify_data.append(temp_verify)
                                output_data.append(temp_output)

            print(d_1, d_2)

    Total_data = np.concatenate([input_data, verify_data, output_data], axis=1)
    pd_Total = pd.DataFrame(Total_data)

    print pd_Total.shape

    return pd_Total

def make_fold(pd_data, fold_num):

    DataNo = pd_data.shape[0]
    input_FeatNo = 5
    verify_FeatNo = 25 + input_FeatNo
    output_FeatNo = 1 + verify_FeatNo
    FoldDataNo = int(DataNo/fold_num)

    total_data = pd_data.iloc[np.random.permutation(pd_data.index)]
    total_data = total_data.T

    print(total_data.shape)

    ## Validation Data set ##
    for i in range(fold_num):
        
        temp_input_Valid = total_data.iloc[:input_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        temp_verify_Valid = total_data.iloc[input_FeatNo:verify_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        temp_output_Valid = total_data.iloc[verify_FeatNo:output_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        
        launch_1 = 'input_Fold%d = temp_input_Valid'%(i+1)
        launch_2 = 'verify_Fold%d = temp_verify_Valid'%(i+1)
        launch_3 = 'output_Fold%d = temp_output_Valid'%(i+1)

        exec(launch_1)
        exec(launch_2)
        exec(launch_3)

    # print(Validation_input_Fold1.shape)
    # print(Validation_verify_Fold1.shape)
    # print(Validation_output_Fold1.shape)

    print('fold data done')

    for i in range(0, fold_num):

        launch_1 = 'save_data_as_csv(input_Fold%d, \'input_Fold_%d\')'%(i+1,i+1)
        launch_2 = 'save_data_as_csv(verify_Fold%d, \'verify_Fold_%d\')'%(i+1,i+1)
        launch_3 = 'save_data_as_csv(output_Fold%d, \'output_Fold_%d\')'%(i+1,i+1)
        
        exec(launch_1)
        exec(launch_2)
        exec(launch_3)

        print(i+1)

    print ('Save data done')

if __name__ == "__main__":

    sensor = ToF_Sensor()

    total_data = make_data(sensor, 50, 20)

    fold_data = make_fold(total_data, 25)