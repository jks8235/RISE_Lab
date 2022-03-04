#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import time
import pdb
import numpy as np
import sys
import pandas as pd
# import rospy
import math
import copy
import random

sys.path.insert(0 , '/home/jee/work_space/catkin_wk/src/RISE_Lab/Library/')
from CoppeliaSim_bluezero import b0RemoteApi

class VrepObjectStateBridge(object):
    def __init__(self, client, obj_handles, object_names):
        self.client = client
        self.obj_handles = obj_handles
        self.obj_name = object_names

        # create pub topics
        self.object_state_pub_topics = dict()
        for name in object_names:
            self.object_state_pub_topics[name] = self.client.simxCreatePublisher(True)

    def excute_pose(self, pose):

        self.client.simxSetObjectPosition(self.obj_handles[self.obj_name[0]], -1, pose[0], self.object_state_pub_topics[self.obj_name[0]])
        self.client.simxSetObjectPosition(self.obj_handles[self.obj_name[1]], -1, pose[1], self.object_state_pub_topics[self.obj_name[1]])

class VrepSensorMountBridge(object):
    def __init__(self, client, obj_handles, object_names):
        self.client = client
        self.obj_handles = obj_handles
        self.obj_name = object_names

        # create pub topics
        self.object_state_pub_topics = dict()
        for name in object_names:
            self.object_state_pub_topics[name] = self.client.simxCreatePublisher(True)

    def excute_pose(self, pose):

        self.client.simxSetObjectPosition(self.obj_handles[self.obj_name[0]], -1, pose, self.object_state_pub_topics[self.obj_name[0]])

class VrepDistanceSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_names):
        # variables
        self.client = client
        self.obj_handles = obj_handles
        self.sensor_names = sensor_names
        self.default_distance = 0.8

        self.FOV_distance = [self.default_distance for i in range(19)]
        self.FOV_object_check = [0 for i in range(19)]
        self.sensor_pos = []

        # create vrep B0 subscriber function
        # Read proximitisensor
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_center, dropMessages=True))

        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[4]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1_4, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[5]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1_5, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[6]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1_6, dropMessages=True))

        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[7]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[8]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[9]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[10]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_4, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[11]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_5, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[12]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_6, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[13]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_7, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[14]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_8, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[15]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_9, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[16]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_10, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[17]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_11, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2_12, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb, dropMessages=True))

    def get_distance(self):
    
        # distance = sum(self.FOV_distance)/len(self.FOV_distance)
        distance = min(self.FOV_distance)

        return distance
    
## Call Back
# Pos Call Back
    def _pos_cb(self, msg):
        self.sensor_pos = msg[1]

# Proxi Sensor Call Back 1
    def _proxi_sensor_cb_center(self, msg):
        if msg[1] == 1:
            self.FOV_distance[0] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[0] = self.default_distance
            self.FOV_object_check[0] = 0

    def _proxi_sensor_cb_1_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance[1] = msg[2]
            self.FOV_object_check[1] = 1
        else:
            self.FOV_distance[1] = self.default_distance
            self.FOV_object_check[1] = 0

    def _proxi_sensor_cb_1_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance[2] = msg[2]
            self.FOV_object_check[2] = 1
        else:
            self.FOV_distance[2] = self.default_distance
            self.FOV_object_check[2] = 0

    def _proxi_sensor_cb_1_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance[3] = msg[2]
            self.FOV_object_check[3] = 1
        else:
            self.FOV_distance[3] = self.default_distance
            self.FOV_object_check[3] = 0

    def _proxi_sensor_cb_1_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance[4] = msg[2]
            self.FOV_object_check[4] = 1
        else:
            self.FOV_distance[4] = self.default_distance
            self.FOV_object_check[4] = 0

    def _proxi_sensor_cb_1_5(self, msg):
        if msg[1] == 1:
            self.FOV_distance[5] = msg[2]
            self.FOV_object_check[5] = 1
        else:
            self.FOV_distance[5] = self.default_distance
            self.FOV_object_check[5] = 0

    def _proxi_sensor_cb_1_6(self, msg):
        if msg[1] == 1:
            self.FOV_distance[6] = msg[2]
            self.FOV_object_check[6] = 1
        else:
            self.FOV_distance[6] = self.default_distance
            self.FOV_object_check[6] = 0

    def _proxi_sensor_cb_2_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance[7] = msg[2]
            self.FOV_object_check[7] = 1
        else:
            self.FOV_distance[7] = self.default_distance
            self.FOV_object_check[7] = 0

    def _proxi_sensor_cb_2_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance[8] = msg[2]
            self.FOV_object_check[8] = 1
        else:
            self.FOV_distance[8] = self.default_distance
            self.FOV_object_check[8] = 0

    def _proxi_sensor_cb_2_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance[9] = msg[2]
            self.FOV_object_check[9] = 1
        else:
            self.FOV_distance[9] = self.default_distance
            self.FOV_object_check[9] = 0

    def _proxi_sensor_cb_2_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance[10] = msg[2]
            self.FOV_object_check[10] = 1
        else:
            self.FOV_distance[10] = self.default_distance
            self.FOV_object_check[10] = 0

    def _proxi_sensor_cb_2_5(self, msg):
        if msg[1] == 1:
            self.FOV_distance[11] = msg[2]
            self.FOV_object_check[11] = 1
        else:
            self.FOV_distance[11] = self.default_distance
            self.FOV_object_check[11] = 0

    def _proxi_sensor_cb_2_6(self, msg):
        if msg[1] == 1:
            self.FOV_distance[12] = msg[2]
            self.FOV_object_check[12] = 1
        else:
            self.FOV_distance[12] = self.default_distance
            self.FOV_object_check[12] = 0

    def _proxi_sensor_cb_2_7(self, msg):
        if msg[1] == 1:
            self.FOV_distance[13] = msg[2]
            self.FOV_object_check[13] = 1
        else:
            self.FOV_distance[13] = self.default_distance
            self.FOV_object_check[13] = 0

    def _proxi_sensor_cb_2_8(self, msg):
        if msg[1] == 1:
            self.FOV_distance[14] = msg[2]
            self.FOV_object_check[14] = 1
        else:
            self.FOV_distance[14] = self.default_distance
            self.FOV_object_check[14] = 0

    def _proxi_sensor_cb_2_9(self, msg):
        if msg[1] == 1:
            self.FOV_distance[15] = msg[2]
            self.FOV_object_check[15] = 1
        else:
            self.FOV_distance[15] = self.default_distance
            self.FOV_object_check[15] = 0

    def _proxi_sensor_cb_2_10(self, msg):
        if msg[1] == 1:
            self.FOV_distance[16] = msg[2]
            self.FOV_object_check[16] = 1
        else:
            self.FOV_distance[16] = self.default_distance
            self.FOV_object_check[16] = 0

    def _proxi_sensor_cb_2_11(self, msg):
        if msg[1] == 1:
            self.FOV_distance[17] = msg[2]
            self.FOV_object_check[17] = 1
        else:
            self.FOV_distance[17] = self.default_distance
            self.FOV_object_check[17] = 0

    def _proxi_sensor_cb_2_12(self, msg):
        if msg[1] == 1:
            self.FOV_distance[18] = msg[2]
            self.FOV_object_check[18] = 1
        else:
            self.FOV_distance[18] = self.default_distance
            self.FOV_object_check[18] = 0

class VrepInterface(object):

    ## Object Names set
    BOX_OBJ_NAMES = [
        "Box1",
        "Box2",
        "Box3"
        ]

    MOUNT_OBJ_NAMES = [
        "Sensor_Body"
        ]

    SENSOR_OBJ_NAMES = [
        "Proximity_sensor_Center",
        "Proximity_sensor_1_1",
        "Proximity_sensor_1_2",
        "Proximity_sensor_1_3",
        "Proximity_sensor_1_4",
        "Proximity_sensor_1_5",
        "Proximity_sensor_1_6",
        "Proximity_sensor_2_01",
        "Proximity_sensor_2_02",
        "Proximity_sensor_2_03",
        "Proximity_sensor_2_04",
        "Proximity_sensor_2_05",
        "Proximity_sensor_2_06",
        "Proximity_sensor_2_07",
        "Proximity_sensor_2_08",
        "Proximity_sensor_2_09",
        "Proximity_sensor_2_10",
        "Proximity_sensor_2_11",
        "Proximity_sensor_2_12",
        ]

    OBJ_NAMES = BOX_OBJ_NAMES + MOUNT_OBJ_NAMES + SENSOR_OBJ_NAMES

    def __init__(self):
        ### Vrep
        # vrep client variables
        self.do_next_step = True

        # connect to vrep server
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi', 'b0RemoteApi')

        # vrep basic option set
        self.get_object_handles()
        self.client.simxSynchronous(True)

        # vrep bridge define
        self.mount = VrepSensorMountBridge(self.client, self.obj_handles, self.MOUNT_OBJ_NAMES)
        self.FOV_distance_sensor = VrepDistanceSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)
        self.Objects = VrepObjectStateBridge(self.client, self.obj_handles, self.BOX_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        ## data
        self.trajectory = []

        # defualt variables
        self.input_data = []
        self.output_data = []
        self.input_temp = []
        self.output_temp = []

        self.learning_input = []
        self.learning_output =[]

        self.flag = True

    def get_object_handles(self):
        self.obj_handles = dict()
        for name in self.OBJ_NAMES:
            self.obj_handles[name] = self.client.simxGetObjectHandle(
                name,
                self.client.simxServiceCall()
                )[1]

    def simulation_step_start_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        ### Write the working code ###
        if len(self.trajectory) == 0:
            self.flag = False
        else:
            self.mount.excute_pose(self.trajectory[0])
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        FOV_distance = self.FOV_distance_sensor.FOV_distance
        object_check = self.FOV_distance_sensor.FOV_object_check
        bundle_pos = self.FOV_distance_sensor.sensor_pos

        sensor_distance = self.FOV_distance_sensor.get_distance()

        if self.flag == False:
        
            # add data set
            self.input_data = self.input_temp
            self.output_data = self.output_temp

            self.make_data()

            # clear temp data
            self.input_temp = []
            self.output_temp = []

        else:
            self.input_temp = self.input_temp + [bundle_pos + [sensor_distance]]
            self.output_temp = self.output_temp + [FOV_distance + object_check]
        
        # change do_next_step state
        self.do_next_step = True

    def start_simulation(self):
        self.client.simxStartSimulation(self.client.simxDefaultPublisher())
        print("start vrep simulation with bluezero remote API")

    def stop_simulation(self):
        self.client.simxStopSimulation(self.client.simxDefaultPublisher())
        print("stop simulation")

    def step_simulation(self):
        while not self.do_next_step:
            self.client.simxSpinOnce()
        self.do_next_step = False
        self.client.simxSynchronousTrigger()

    def set_trajectory(self, start_pose, end_pose, step=20):

        self.trajectory = []
        step_pose = [((end_pose[i]-start_pose[i])/step) for i in range(3)]

        for pos_num in range(step+1):
            self.trajectory.append([(start_pose[i]+pos_num*step_pose[i]) for i in range(3)])

        self.flag = True

    def set_object_position(self, obj_pos):
        self.Objects.excute_pose(obj_pos)

    def make_data(self):

        self.learning_input = []
        self.learning_output = []

        step = 350
        data_step = 21

        for i in range(len(self.input_data)-20):
            temp_input = self.input_data[i:(i+21)]
            temp_output = self.output_data[(i+20)]

            # print(temp_output)

            self.learning_input += [sum(temp_input, [])]
            self.learning_output += [temp_output]

def save_as_npy(data, name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_2D/data/hexagon_middle_sonar/learning_set_1/' + name + '.npy'
    np.save(path, data)

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def make_path_set(resolution):
    resolution = int(resolution*1000)
    x_range = range(-500, 500+1, resolution)
    y_range = range(-500, 500+1, resolution)
    # print(x_range)
    # print(y_range)
    # z_range = range(-500, 500+1, resolution)

    start_point = []
    end_point = []

    for x_start in x_range:
        for x_end in x_range:
            for y_start in y_range:
                for y_end in y_range:

                    start_point += [[float(x_start)/1000, float(y_start)/1000, 0.85]]
                    end_point += [[float(x_end)/1000, float(y_end)/1000, 0.85]]

    # print(len(start_point))

    position_path = zip(start_point, end_point)

    return position_path

def make_object_pos_set(resolution):
    # input unit (m), calculation unit (mm), output unit (m)
    resolution = int(resolution*1000)

    # situation
    Box_1_z_range = range(0, 650, resolution)

    Box_2_z_range = range(0, 650, resolution)

    ### Box poses set ###
    Box_1_poses = []
    for Box_1_z in Box_1_z_range:
        Box_1_poses += [[0.0, 0.5, Box_1_z/1000.0]]

    Box_2_poses = []
    for Box_2_z in Box_2_z_range:
        Box_2_poses += [[0.0, -0.5, Box_2_z/1000.0]]

    ### Total Box poses ##
    obj_pos = []
    for Box_1_pos in Box_1_poses:
        for Box_2_pos in Box_2_poses:
            obj_pos += [[Box_1_pos, Box_2_pos]]

    ### return ###
    return obj_pos


if __name__ == '__main__':
    input_data = []
    output_data = []

    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    save_count = 1

#----------------Learning Data-----------------------

    position_path = make_path_set(0.2)
    obj_poses = make_object_pos_set(0.05)

    data_num = len(obj_poses)
    print(data_num)

    for obj_pos in obj_poses:
        for start_angle, end_angle in position_path:
            pass
            vrep.set_object_position(obj_pos)
            vrep.set_trajectory(start_angle, end_angle, step=100)
            
            while vrep.flag:
                vrep.step_simulation()

            input_data += copy.deepcopy(vrep.learning_input)
            output_data += copy.deepcopy(vrep.learning_output)

        count += 1
        print("%d/%d"%(count, data_num))

        input_np_data = np.array(input_data)
        output_np_data = np.array(output_data)

        save_as_npy(input_np_data, 'input_Fold_%d'%(save_count))
        save_as_npy(output_np_data, 'output_Fold_%d'%(save_count))

        input_data = []
        output_data = []

        save_count += 1
#-----------------------Test Data---------------------------------

    # position_path = make_path_set()
    # data_num = (len(position_path))
    # print(data_num)

    # for start_angle, end_angle in position_path:

    #     vrep.set_trajectory(start_angle, end_angle, step=350)
        
    #     while vrep.flag:
    #         vrep.step_simulation()

    #     input_data += copy.deepcopy(vrep.learning_input)
    #     output_data += copy.deepcopy(vrep.learning_output)

    #     count += 1
    #     print("%d/%d")%(count, data_num)

    # input_np_data = pd.DataFrame(np.array(input_data))
    # output_np_data = pd.DataFrame(np.array(output_data))

    # save_as_npy(input_np_data, 'input_Fold_%d'%(save_count))
    # save_as_npy(output_np_data, 'output_Fold_%d'%(save_count))
#--------------------------------------------------------------------

    vrep.stop_simulation()

    print(input_np_data.shape)
    print(output_np_data.shape)