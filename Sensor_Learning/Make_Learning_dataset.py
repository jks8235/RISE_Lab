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


class VrepRobotStateBridge(object):
    def __init__(self, client, obj_handles, joint_names):
        self.client = client
        self.obj_handles = obj_handles
        self.joint_names = joint_names

        self.joint_position = [0.0 for i in range(6)]

        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[0]], self.client.simxCreateSubscriber(self._joint1_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[1]], self.client.simxCreateSubscriber(self._joint2_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[2]], self.client.simxCreateSubscriber(self._joint3_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[3]], self.client.simxCreateSubscriber(self._joint4_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[4]], self.client.simxCreateSubscriber(self._joint5_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[5]], self.client.simxCreateSubscriber(self._joint6_cb, dropMessages=True))

        # create pub topics
        self.joint_state_pub_topics = dict()
        for name in joint_names:
            self.joint_state_pub_topics[name] = self.client.simxCreatePublisher(True)

    def excute_pose(self, pose):
        bundle = zip(self.joint_names, pose)
        for name, desired_position in bundle:
            self.client.simxSetJointTargetPosition(self.obj_handles[name], desired_position, self.joint_state_pub_topics[name])

## Call Back functions
    # Joint Postion Call Back
    def _joint1_cb(self, msg):
        self.joint_position[0] = msg[1]

    def _joint2_cb(self, msg):
        self.joint_position[1] = msg[1]

    def _joint3_cb(self, msg):
        self.joint_position[2] = msg[1]

    def _joint4_cb(self, msg):
        self.joint_position[3] = msg[1]

    def _joint5_cb(self, msg):
        self.joint_position[4] = msg[1]

    def _joint6_cb(self, msg):
        self.joint_position[5] = msg[1]


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
        self.client.simxSetObjectPosition(self.obj_handles[self.obj_name[2]], -1, pose[2], self.object_state_pub_topics[self.obj_name[2]])

class VrepDistanceSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_names):
        # variables
        self.client = client
        self.obj_handles = obj_handles
        self.sensor_names = sensor_names
        self.default_distance = 0.9
        self.FOV_distance = [self.default_distance for i in range(len(self.sensor_names))]
        self.FOV_object_check = [0 for i in range(len(self.sensor_names))]
        self.pos = []

        # create vrep B0 subscriber function
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._distance_cb_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._distance_cb_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._distance_cb_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._distance_cb_4, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[4]], self.client.simxCreateSubscriber(self._distance_cb_5, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[5]], self.client.simxCreateSubscriber(self._distance_cb_6, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[6]], self.client.simxCreateSubscriber(self._distance_cb_7, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[7]], self.client.simxCreateSubscriber(self._distance_cb_8, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[8]], self.client.simxCreateSubscriber(self._distance_cb_9, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[9]], self.client.simxCreateSubscriber(self._distance_cb_10, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[10]], self.client.simxCreateSubscriber(self._distance_cb_11, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[11]], self.client.simxCreateSubscriber(self._distance_cb_12, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[12]], self.client.simxCreateSubscriber(self._distance_cb_13, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[13]], self.client.simxCreateSubscriber(self._distance_cb_14, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[14]], self.client.simxCreateSubscriber(self._distance_cb_15, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[15]], self.client.simxCreateSubscriber(self._distance_cb_16, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[16]], self.client.simxCreateSubscriber(self._distance_cb_17, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb, dropMessages=True))

    def get_distance(self):
        FOV_distance = self.FOV_distance
        AVG_distance = sum(self.FOV_distance)/len(self.FOV_distance)

        return AVG_distance, FOV_distance
    
## Call Back
    # Pos Call Back
    def _pos_cb(self, msg):
        self.pos = msg[1]

    # Distance Call Back
    def _distance_cb_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance[0] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[0] = self.default_distance
            self.FOV_object_check[0] = 0
            
    def _distance_cb_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance[1] = msg[2]
            self.FOV_object_check[1] = 1
        else:
            self.FOV_distance[1] = self.default_distance
            self.FOV_object_check[1] = 0
            
    def _distance_cb_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance[2] = msg[2]
            self.FOV_object_check[2] = 1
        else:
            self.FOV_distance[2] = self.default_distance
            self.FOV_object_check[2] = 0
            
    def _distance_cb_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance[3] = msg[2]
            self.FOV_object_check[3] = 1
        else:
            self.FOV_distance[3] = self.default_distance
            self.FOV_object_check[3] = 0
            
    def _distance_cb_5(self, msg):
        if msg[1] == 1:
            self.FOV_distance[4] = msg[2]
            self.FOV_object_check[4] = 1
        else:
            self.FOV_distance[4] = self.default_distance
            self.FOV_object_check[4] = 0
            
    def _distance_cb_6(self, msg):
        if msg[1] == 1:
            self.FOV_distance[5] = msg[2]
            self.FOV_object_check[5] = 1
        else:
            self.FOV_distance[5] = self.default_distance
            self.FOV_object_check[5] = 0
            
    def _distance_cb_7(self, msg):
        if msg[1] == 1:
            self.FOV_distance[6] = msg[2]
            self.FOV_object_check[6] = 1
        else:
            self.FOV_distance[6] = self.default_distance
            self.FOV_object_check[6] = 0
            
    def _distance_cb_8(self, msg):
        if msg[1] == 1:
            self.FOV_distance[7] = msg[2]
            self.FOV_object_check[7] = 1
        else:
            self.FOV_distance[7] = self.default_distance
            self.FOV_object_check[7] = 0
            
    def _distance_cb_9(self, msg):
        if msg[1] == 1:
            self.FOV_distance[8] = msg[2]
            self.FOV_object_check[8] = 1
        else:
            self.FOV_distance[8] = self.default_distance
            self.FOV_object_check[8] = 0
            
    def _distance_cb_10(self, msg):
        if msg[1] == 1:
            self.FOV_distance[9] = msg[2]
            self.FOV_object_check[9] = 1
        else:
            self.FOV_distance[9] = self.default_distance
            self.FOV_object_check[9] = 0
            
    def _distance_cb_11(self, msg):
        if msg[1] == 1:
            self.FOV_distance[10] = msg[2]
            self.FOV_object_check[10] = 1
        else:
            self.FOV_distance[10] = self.default_distance
            self.FOV_object_check[10] = 0
            
    def _distance_cb_12(self, msg):
        if msg[1] == 1:
            self.FOV_distance[11] = msg[2]
            self.FOV_object_check[11] = 1
        else:
            self.FOV_distance[11] = self.default_distance
            self.FOV_object_check[11] = 0
            
    def _distance_cb_13(self, msg):
        if msg[1] == 1:
            self.FOV_distance[12] = msg[2]
            self.FOV_object_check[12] = 1
        else:
            self.FOV_distance[12] = self.default_distance
            self.FOV_object_check[12] = 0
            
    def _distance_cb_14(self, msg):
        if msg[1] == 1:
            self.FOV_distance[13] = msg[2]
            self.FOV_object_check[13] = 1
        else:
            self.FOV_distance[13] = self.default_distance
            self.FOV_object_check[13] = 0
            
    def _distance_cb_15(self, msg):
        if msg[1] == 1:
            self.FOV_distance[14] = msg[2]
            self.FOV_object_check[14] = 1
        else:
            self.FOV_distance[14] = self.default_distance
            self.FOV_object_check[14] = 0
            
    def _distance_cb_16(self, msg):
        if msg[1] == 1:
            self.FOV_distance[15] = msg[2]
            self.FOV_object_check[15] = 1
        else:
            self.FOV_distance[15] = self.default_distance
            self.FOV_object_check[15] = 0
            
    def _distance_cb_17(self, msg):
        if msg[1] == 1:
            self.FOV_distance[16] = msg[2]
            self.FOV_object_check[16] = 1
        else:
            self.FOV_distance[16] = self.default_distance
            self.FOV_object_check[16] = 0
            

class VrepInterface(object):

## Object Names set
    BOX_OBJ_NAMES = [
        "Box1",
        "Box2",
        "Box3"
        ]

    INDY_OBJ_NAMES = [
        "joint0",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        ]

    SENSOR_OBJ_NAMES = [
        "Proximity_sensor1",
        "Proximity_sensor2",
        "Proximity_sensor3",
        "Proximity_sensor4",
        "Proximity_sensor5",
        "Proximity_sensor6",
        "Proximity_sensor7",
        "Proximity_sensor8",
        "Proximity_sensor9",
        "Proximity_sensor10",
        "Proximity_sensor11",
        "Proximity_sensor12",
        "Proximity_sensor13",
        "Proximity_sensor14",
        "Proximity_sensor15",
        "Proximity_sensor16",
        "Proximity_sensor17"
        ]

    OBJ_NAMES = BOX_OBJ_NAMES + INDY_OBJ_NAMES + SENSOR_OBJ_NAMES

#
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
        self.indy = VrepRobotStateBridge(self.client, self.obj_handles, self.INDY_OBJ_NAMES)
        self.FOV_distance_sensor = VrepDistanceSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)
        self.Objects = VrepObjectStateBridge(self.client, self.obj_handles, self.BOX_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        # defualt variables
        self.input_data =[]
        self.output_data = []
        self.input_temp = []
        self.output_temp = []
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
            self.indy.excute_pose(self.trajectory[0])
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        AVG_distance, FOV_distance = self.FOV_distance_sensor.get_distance()
        bundle_pos = self.FOV_distance_sensor.pos

        if self.flag == False:
            # self.output_temp = FOV_distance

            # add data set
            self.input_data = self.input_temp
            self.output_data = self.output_temp

            # clear temp data
            self.input_temp = []
            self.output_temp = []

        else:
            self.input_temp = self.input_temp + bundle_pos + [AVG_distance]
            self.output_temp = FOV_distance + self.FOV_distance_sensor.FOV_object_check

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
        step_pose = [((end_pose[i]-start_pose[i])/step) for i in range(6)]

        for pos_num in range(step+1):
            self.trajectory.append([deg2rad(start_pose[i]+pos_num*step_pose[i]) for i in range(6)])

        self.flag = True

    def set_object_position(self, obj_pos):
        self.Objects.excute_pose(obj_pos)

def _save_data_as_csv(data, name):
    path = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Learning_situation_1' + name + '.csv'
    data.to_csv(path, sep=',', header=None, index=None)

def make_fold(pd_data, fold_num):

    DataNo = pd_data.shape[0]
    input_FeatNo = 168
    output_FeatNo = 17 + input_FeatNo
    FoldDataNo = int(DataNo/fold_num)

    total_data = pd_data

    # total_data = pd_data.iloc[np.random.permutation(pd_data.index)]
    # total_data = total_data.T
    
    print(total_data.shape)

    ## Validation Data set ##
    for i in range(fold_num):
        
        temp_input_Valid = total_data.iloc[:input_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        temp_output_Valid = total_data.iloc[input_FeatNo:output_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        
        launch_1 = 'input_Fold%d = temp_input_Valid'%(i+1)
        launch_2 = 'output_Fold%d = temp_output_Valid'%(i+1)

        exec(launch_1)
        exec(launch_2)

    print(input_Fold1.shape)
    print(output_Fold1.shape)
    # print(output_Fold1)

    print('fold data done')

    for i in range(0, fold_num):

        launch_1 = '_save_data_as_csv(input_Fold%d, \'input_Fold_%d\')'%(i+1,i+1)
        launch_2 = '_save_data_as_csv(output_Fold%d, \'output_Fold_%d\')'%(i+1,i+1)
        
        exec(launch_1)
        exec(launch_2)

        print(i+1)

    print ('Save data done')

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def make_path_set(resolution):
    theta_0_range = range(-150, 150+1 ,resolution)
    theta_2_range = range(-120, -60+1, resolution)

    start_point = []
    end_point = []

    for theta_0_start in theta_0_range:
        for theta_0_end in theta_0_range:
            for theta_2_start in theta_2_range:
                for theta_2_end in theta_2_range:
                    start_point += [[float(theta_0_start), 0.0, float(theta_2_start), 0.0, 0.0, 0.0]]
                    end_point += [[float(theta_0_end), 0.0, float(theta_2_end), 0.0, 0.0, 0.0]]

    angle_path = zip(start_point, end_point)

    return angle_path

def make_object_pos_set(resolution):
    # input unit (m), calculation unit (mm), output unit (m)
    resolution = int(resolution*1000)

### situation 1 ###
    Box_1_x_range = [-1100]
    Box_1_y_range = range(-1100, 0+1, resolution*2)
    Box_1_z_range = [400]
    
    Box_2_x_range = range(900, 1100+1, resolution)
    Box_2_y_range = range(-800, 800+1, resolution*2)
    Box_2_z_range = [300]

    Box_3_x_range = range(-1100, 1100+1, resolution*3)
    Box_3_y_range = [1300]
    Box_3_z_range = [900]

### Box poses set ###
    Box_1_poses = []
    for Box_1_x in Box_1_x_range:
        for Box_1_y in Box_1_y_range:
            for Box_1_z in Box_1_z_range:
                Box_1_poses += [Box_1_x/1000.0, Box_1_y/1000.0, Box_1_z/1000.0]

    Box_2_poses = []
    for Box_2_x in Box_2_x_range:
        for Box_2_y in Box_2_y_range:
            for Box_2_z in Box_2_z_range:
                Box_2_poses += [[Box_2_x/1000.0, Box_2_y/1000.0, Box_2_z/1000.0]]

    Box_3_poses = []
    for Box_3_x in Box_3_x_range:
        for Box_3_y in Box_3_y_range:
            for Box_3_z in Box_3_z_range:
                Box_3_poses += [[Box_3_x/1000.0, Box_3_y/1000.0, Box_3_z/1000.0]]

### Total Box poses ##
    obj_pos = []
    for Box_1_pos in Box_1_poses:
        for Box_2_pos in Box_2_poses:
            for Box_3_pos in Box_3_poses:
                obj_pos += [[Box_1_pos, Box_2_pos, Box_3_pos]]

### return ###
    return obj_pos


if __name__ == '__main__':
    input_data = []
    output_data = []

    angle_path = make_path_set(45)
    obj_poses = make_object_pos_set(0.2)

    print(len(angle_path), len(obj_poses))
    data_num = (len(angle_path)*len(obj_poses))
    print(data_num)
    
    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0

    for obj_pos in obj_poses:
        for start_angle, end_angle in angle_path:

            count += 1
            print(count)

            vrep.set_object_position(obj_pos)
            vrep.set_trajectory(start_angle, end_angle)
            
            while vrep.flag:
                vrep.step_simulation()

            input_data += vrep.input_data
            output_data += vrep.output_data

    vrep.stop_simulation()

    input_np_data = np.reshape(input_data,(data_num,168))
    output_np_data = np.reshape(output_data,(data_num,34))

    print(input_np_data.shape)
    print(output_np_data.shape)

    Total_data = np.concatenate([input_np_data, output_np_data], axis=1)
    pd_Total = pd.DataFrame(Total_data).T

    print(pd_Total.shape)

    make_fold(pd_Total,8)
