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
        self.default_distance = 0.8

        self.FOV_distance = [0]
        self.FOV_object_check = [0]
        self.sensor_pos = []

        # create vrep B0 subscriber function
        # Read proximitisensor
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_center, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb, dropMessages=True))
    
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
        "Proximity_sensor"
        ]

    OBJ_NAMES = BOX_OBJ_NAMES + INDY_OBJ_NAMES + SENSOR_OBJ_NAMES

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
        print(self.OBJ_NAMES)
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
            # print("move left", len(self.trajectory), self.trajectory[0])
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        FOV_distance = self.FOV_distance_sensor.FOV_distance
        object_check = self.FOV_distance_sensor.FOV_object_check
        bundle_pos = self.FOV_distance_sensor.sensor_pos

        if self.flag == False:
        
            # add data set
            self.input_data = self.input_temp
            self.output_data = self.output_temp

            self.make_data()

            # clear temp data
            self.input_temp = []
            self.output_temp = []

        else:
            self.input_temp = self.input_temp + [bundle_pos + FOV_distance]
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
        step_pose = [((end_pose[i]-start_pose[i])/step) for i in range(6)]
        # print(step_pose)

        for pos_num in range(step+1):
            self.trajectory.append([deg2rad(start_pose[i]+pos_num*step_pose[i]) for i in range(6)])

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
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/sonar/test_1/' + name + '.npy'
    np.save(path, data)

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def make_path_set():

    theta_0_range = range(-175, 175+1 ,350) # -175 ~ 175
    # theta_1_range = range(-175, 175+1 ,resolution) # -175 ~ 175 
    theta_2_range = range(-120, 120+1, 240) # -175 ~ 175
    theta_3_range = range(-150, 150+1 ,150) # -175 ~ 175
    theta_4_range = range(-120, 120+1 ,120) # -175 ~ 175
    # theta_5_range = range(-215, 215+1 ,resolution) # -215 ~ 215

    start_point = []
    end_point = []

    for theta_0_start in theta_0_range:
        for theta_0_end in theta_0_range:
            for theta_2_start in theta_2_range:
                for theta_2_end in theta_2_range:
                    for theta_3_start in theta_3_range:
                        for theta_3_end in theta_3_range:
                            for theta_4_start in theta_4_range:
                                for theta_4_end in theta_4_range:
                                            
                                    start_point += [[float(theta_0_start), 0.0, float(theta_2_start), float(theta_3_start), float(theta_4_start), 0.0]]
                                    end_point += [[float(theta_0_end), 0.0, float(theta_2_end), float(theta_3_end), float(theta_4_end), 0.0]]

    angle_path = zip(start_point, end_point)

    return angle_path

def make_object_pos_set(resolution):
    # input unit (m), calculation unit (mm), output unit (m)
    resolution = int(resolution*1000)

    # situation
    Box_1_x_range = range(-1100, -800, resolution)
    Box_1_y_range = range(-1100, 0+1, resolution*2)
    Box_1_z_range = [400]
    
    Box_2_x_range = range(900, 1100+1, resolution)
    Box_2_y_range = range(-800, 800+1, resolution*4)
    Box_2_z_range = [300]

    Box_3_x_range = range(-1100, 1100+1, resolution*5)
    Box_3_y_range = range(800, 1300, resolution)
    Box_3_z_range = [900]

    ### Box poses set ###
    Box_1_poses = []
    for Box_1_x in Box_1_x_range:
        for Box_1_y in Box_1_y_range:
            for Box_1_z in Box_1_z_range:
                Box_1_poses += [[Box_1_x/1000.0, Box_1_y/1000.0, Box_1_z/1000.0]]

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

    
    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    save_count = 1

#---------------------------------------------------------
    # # learning data

    # angle_path = make_path_set()
    # obj_poses = make_object_pos_set(0.25)

    # print(len(angle_path), len(obj_poses))
    # data_num = (len(angle_path)*len(obj_poses))
    # print(data_num)

    # for obj_pos in obj_poses:
    #     for start_angle, end_angle in angle_path:

    #         vrep.set_object_position(obj_pos)
    #         vrep.set_trajectory(start_angle, end_angle, step=350)
            
    #         while vrep.flag:
    #             vrep.step_simulation()

    #         input_data += copy.deepcopy(vrep.learning_input)
    #         output_data += copy.deepcopy(vrep.learning_output)

    #         count += 1
    #         print("%d/%d")%(count, data_num)

    #     input_np_data = np.array(input_data)
    #     output_np_data = np.array(output_data)

    #     save_as_npy(input_np_data, 'input_Fold_%d'%(save_count))
    #     save_as_npy(output_np_data, 'output_Fold_%d'%(save_count))

    #     input_data = []
    #     output_data = []

    #     save_count += 1
#----------------------------------------------------------------------
    # Test data

    angle_path = make_path_set()
    data_num = (len(angle_path))
    print(data_num)

    for start_angle, end_angle in angle_path:

        vrep.set_trajectory(start_angle, end_angle, step=350)
        
        while vrep.flag:
            vrep.step_simulation()

        input_data += copy.deepcopy(vrep.learning_input)
        output_data += copy.deepcopy(vrep.learning_output)

        count += 1
        print("%d/%d")%(count, data_num)
        # print(input_data, output_data)

    input_np_data = np.array(input_data)
    output_np_data = np.array(output_data)

    save_as_npy(input_np_data, 'input_Fold_%d'%(save_count))
    save_as_npy(output_np_data, 'output_Fold_%d'%(save_count))
#--------------------------------------------------------------------

    vrep.stop_simulation()

    print(input_np_data.shape)
    print(output_np_data.shape)