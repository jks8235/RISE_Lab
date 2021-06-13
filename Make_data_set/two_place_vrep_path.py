#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import pandas as pd
import numpy as np
# import rospy
import time
import math
import copy
import tf
import sys
import random

sys.path.insert(0 , '/home/jee/catkin_ws/src/RISE_Lab/Library/CoppeliaSim_Lib')
from vrepsim import VrepSimulation


class sensor_bundle():
    def __init__(self):
        self.objacts = [
            'ConcretBlock1',
            'ConcretBlock2',
            'sensor_bundle',
            'Proximity_sensor1',
            'Proximity_sensor2',
            'Proximity_sensor3',
            'Proximity_sensor4',
            'Proximity_sensor5',
            'Proximity_sensor6',
            'Proximity_sensor7',
            'Proximity_sensor8',
            'Proximity_sensor9',
            'Proximity_sensor10',
            'Proximity_sensor11',
            'Proximity_sensor12',
            'Proximity_sensor13',
            'Proximity_sensor14',
            'Proximity_sensor15',
            'Proximity_sensor16',
            'Proximity_sensor17',
            'Proximity_sensor18',
            'Proximity_sensor19',
            'Proximity_sensor20',
            'Proximity_sensor21',
            'Proximity_sensor22',
            'Proximity_sensor23',
            'Proximity_sensor24',
            'Proximity_sensor25',]
        
        self.vrep = VrepSimulation(self.objacts)

        self.bundle_Pose = [0.0, 0.0, 0.45]
        # self.bundle_Rolling = [0.0, 0.0, 0.0]
        # self.bundle_Position = self.bundle_Pose + self.bundle_Rolling
        self.Step_size = 0.02

        self.sensor_name = []
        self.sensor_obj = []
        self.br = tf.TransformBroadcaster()
        self.sensor_num = 25

        for sensor in self.objacts:
            if 'Proximity_sensor' in sensor:
                self.sensor_name.append(sensor)

    def Get_Sensor_Data(self):

        sensor_data = [0 for _ in range(0,self.sensor_num)]

        for sensor in self.sensor_name:
            
            detected_point = self.vrep.get_sona_data(sensor)
            distance = math.sqrt(detected_point[0]*detected_point[0]+detected_point[1]*detected_point[1]+detected_point[2]*detected_point[2])
            sensor_num = self.sensor_name.index(sensor)

            sensor_data[sensor_num] = copy.deepcopy(distance)

        return sensor_data
    
    def Get_Object_Position(self, objname):

        position = self.vrep.get_object_position(objname)
        return position

    def Set_Object_Position(self, objname, position):
        self.vrep.set_object_target_position(objname, position)

    def random_step(self, Pose):

        random_ratio = [random.randrange(-100,101), random.randrange(-100,101), random.randrange(-100,101)]
        del_position = scaling(random_ratio)

        Pose = [round((i + j*self.Step_size),5) for i, j in zip(Pose, del_position)]

        # print(random_ratio)
        # print(del_position)
        # print(Pose)

        return Pose

    def fixed_step(self, Pose, Direction):

        direction_ratio = Direction
        del_position = scaling(direction_ratio)

        Pose = [round((i + j*self.Step_size),5) for i, j in zip(Pose, del_position)]

        print(direction_ratio)
        print(del_position)
        print(Pose)

        return Pose

    def Run(self):
        if self.vrep.is_not_ready:
            raise NotImplementedError
        else:
            self.vrep.start_simulation()
            # self.Fixed_Work()
            self.Random_Work()
            self.vrep.stop_simulation()

    def Get_Bundle_MoveRange(self, resolution = 2):

        Long_Wall_Position = self.Get_Object_Position('ConcretBlock2')
        Short_Wall_Position = self.Get_Object_Position('ConcretBlock1')
        Bundle_Position = self.Get_Object_Position('sensor_bundle')

        # change unit vrep to code (m to cm)
        d_1 = int((Bundle_Position[0] - Short_Wall_Position[0])*100)
        d_2 = int((Bundle_Position[0] - Long_Wall_Position[0])*100)

        plane_max_range = int(d_1*math.tan(deg2rad(12.5))+2)

        y_range = range(-plane_max_range,plane_max_range+1,resolution)
        z_range = range(45-plane_max_range,45+plane_max_range+1,resolution)

        return y_range, z_range
                                                                     
    def Random_Work(self):

        input_data = []
        output_data = []

        iter = 0
        wall_resolution = 10

        Long_Wall_Range = range(-80,-30+1,wall_resolution)
        for d_long in Long_Wall_Range:
            Short_Wall_Range = range(d_long+wall_resolution,-20+1,wall_resolution)
            for d_short in Short_Wall_Range:

                self.Set_Object_Position('ConcretBlock2',[float(d_long)/100, 0.0, 0.45])
                self.Set_Object_Position('ConcretBlock1',[float(d_short)/100, 0.225, 0.225])
                # print(d_long, d_short)
    
                y_range, z_range = self.Get_Bundle_MoveRange(5)

                for y in y_range:
                    for z in z_range:
                        for num in range(10):

                            bundle_position = [0.0, float(y)/100, float(z)/100]
                            distance = np.mean(self.Get_Sensor_Data())

                            temp_input = bundle_position + [distance]
                            
                            step_num = 19

                            for step in range(step_num):
                                bundle_position = self.random_step(bundle_position)

                                self.Set_Object_Position('sensor_bundle', bundle_position)
                                distance = np.mean(self.Get_Sensor_Data())

                                temp_input = temp_input + bundle_position + [distance]

                                print('step %d'%(step+1))

                            temp_output = self.Get_Sensor_Data()
                            input_data.append(temp_input)
                            output_data.append(temp_output)
                        
                            iter += 1
                            print(iter)
                            print('input : ', len(temp_input), temp_input)
                            print('output : ', len(temp_output), temp_output)


        print(input_data)
        print(output_data)

        Total_data = np.concatenate([input_data, output_data], axis=1)
        pd_Total = pd.DataFrame(Total_data)

        make_fold(pd_Total,1)

    # def Fixed_Work(self):

    #         input_data = []
    #         output_data = []

    #         iter = 0
    #         wall_resolution = 20

    #         Long_Wall_Range = range(-80,-30+1,wall_resolution)
    #         for d_long in Long_Wall_Range:
    #             Short_Wall_Range = range(d_long+wall_resolution,-20+1,wall_resolution)
    #             for d_short in Short_Wall_Range:

    #                 self.Set_Object_Position('ConcretBlock2',[float(d_long)/100, 0.0, 0.45])
    #                 self.Set_Object_Position('ConcretBlock1',[float(d_short)/100, 0.225, 0.225])
    #                 # print(d_long, d_short)
        
    #                 y_range, z_range = self.Get_Bundle_MoveRange(5)

    #                 for y in y_range:
    #                     for z in z_range:

    #                         move_distance = 2
                    
    #                         for delta_x in range(-move_distance, move_distance+1, 1):
    #                             for delta_y in range(-(move_distance-abs(delta_x)), (move_distance-abs(delta_x))+1, 1):
    #                                 for delta_z in range(-(move_distance-abs(delta_x)-abs(delta_y)), (move_distance-abs(delta_x)-abs(delta_y))+1, 1):

    #                                     direction = [delta_x, delta_y, delta_z]

    #                                     print(direction)
    #                                     bundle_position = [0.0, float(y)/100, float(z)/100]
    #                                     distance = np.mean(self.Get_Sensor_Data())

    #                                     temp_input = bundle_position + [distance]
                                        
    #                                     step_num = 19

    #                                     for step in range(step_num):
    #                                         bundle_position = self.fixed_step(bundle_position, direction)

    #                                         self.Set_Object_Position('sensor_bundle', bundle_position)
    #                                         distance = np.mean(self.Get_Sensor_Data())

    #                                         temp_input = temp_input + bundle_position + [distance]

    #                                         print('step %d'%(step+1))

    #                                     temp_output = self.Get_Sensor_Data()
    #                                     input_data.append(temp_input)
    #                                     output_data.append(temp_output)
                                    
    #                                     iter += 1
    #                                     print(iter)
    #                                     print('input : ', len(temp_input), temp_input)
    #                                     print('output : ', len(temp_output), temp_output)


    #         print(input_data)
    #         print(output_data)

    #         Total_data = np.concatenate([input_data, output_data], axis=1)
    #         pd_Total = pd.DataFrame(Total_data)

    #         make_fold(pd_Total,1)

def deg2rad(deg):
    return deg/180.0*math.pi

def save_data_as_csv(data, name):
    path = '/home/jee/catkin_ws/src/RISE_Lab/Make_data_set/data/' + name + '.csv'
    data.to_csv(path, sep=',', header=None, index=None)

def make_fold(pd_data, fold_num):

    DataNo = pd_data.shape[0]
    input_FeatNo = 80
    output_FeatNo = 25 + input_FeatNo
    FoldDataNo = int(DataNo/fold_num)

    total_data = pd_data.iloc[np.random.permutation(pd_data.index)]
    total_data = total_data.T
    
    print(total_data.shape)

    ## Validation Data set ##
    for i in range(fold_num):
        
        temp_input_Valid = total_data.iloc[:input_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        temp_output_Valid = total_data.iloc[input_FeatNo:output_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        
        launch_1 = 'input_Fold%d = temp_input_Valid'%(i+1)
        launch_2 = 'output_Fold%d = temp_output_Valid'%(i+1)

        exec(launch_1)
        exec(launch_2)

    # print(Validation_input_Fold1.shape)
    # print(Validation_verify_Fold1.shape)
    # print(Validation_output_Fold1.shape)

    print('fold data done')

    for i in range(0, fold_num):

        launch_1 = 'save_data_as_csv(input_Fold%d, \'Fixed_path_input_Fold_%d\')'%(i+1,i+1)
        launch_2 = 'save_data_as_csv(output_Fold%d, \'Fixed_path_output_Fold_%d\')'%(i+1,i+1)
        
        exec(launch_1)
        exec(launch_2)

        print(i+1)

    print ('Save data done')

def scaling(list_data):
    list_num = len(list_data)
    absoute_sum = 0
    # scalled_data = [0 for num in range(list_num)]
    for i in list_data:
        absoute_sum += abs(i)

    scalled_data = [float(list_data[num])/float(absoute_sum) for num in range(list_num)]

    return scalled_data

if __name__ == "__main__":

    bundle = sensor_bundle()

    if bundle.vrep.is_not_ready:
        raise NotImplementedError
    else:
        bundle.Run()