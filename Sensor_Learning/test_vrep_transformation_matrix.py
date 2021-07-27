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
        self.transformation_matrix = [0 for i in range(len(self.sensor_names))]

        ## create vrep B0 subscriber function
        # Read proximitisensor
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

        # get transformation
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._matrix_cb_1, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[1]], -1, self.client.simxCreateSubscriber(self._matrix_cb_2, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[2]], -1, self.client.simxCreateSubscriber(self._matrix_cb_3, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[3]], -1, self.client.simxCreateSubscriber(self._matrix_cb_4, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[4]], -1, self.client.simxCreateSubscriber(self._matrix_cb_5, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[5]], -1, self.client.simxCreateSubscriber(self._matrix_cb_6, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[6]], -1, self.client.simxCreateSubscriber(self._matrix_cb_7, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[7]], -1, self.client.simxCreateSubscriber(self._matrix_cb_8, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[8]], -1, self.client.simxCreateSubscriber(self._matrix_cb_9, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[9]], -1, self.client.simxCreateSubscriber(self._matrix_cb_10, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[10]], -1, self.client.simxCreateSubscriber(self._matrix_cb_11, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[11]], -1, self.client.simxCreateSubscriber(self._matrix_cb_12, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[12]], -1, self.client.simxCreateSubscriber(self._matrix_cb_13, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[13]], -1, self.client.simxCreateSubscriber(self._matrix_cb_14, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[14]], -1, self.client.simxCreateSubscriber(self._matrix_cb_15, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[15]], -1, self.client.simxCreateSubscriber(self._matrix_cb_16, dropMessages=True))
        self.client.simxGetObjectMatrix(self.obj_handles[self.sensor_names[16]], -1, self.client.simxCreateSubscriber(self._matrix_cb_17, dropMessages=True))

        # get pos
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

# Transformation matrix Call Back
    def _matrix_cb_1(self, msg):
        self.transformation_matrix[0] = msg[1]

    def _matrix_cb_2(self, msg):
        self.transformation_matrix[1] = msg[1]

    def _matrix_cb_3(self, msg):
        self.transformation_matrix[2] = msg[1]

    def _matrix_cb_4(self, msg):
        self.transformation_matrix[3] = msg[1]

    def _matrix_cb_5(self, msg):
        self.transformation_matrix[4] = msg[1]

    def _matrix_cb_6(self, msg):
        self.transformation_matrix[5] = msg[1]
            
    def _matrix_cb_7(self, msg):
        self.transformation_matrix[6] = msg[1]

    def _matrix_cb_8(self, msg):
        self.transformation_matrix[7] = msg[1]

    def _matrix_cb_9(self, msg):
        self.transformation_matrix[8] = msg[1]

    def _matrix_cb_10(self, msg):
        self.transformation_matrix[9] = msg[1]

    def _matrix_cb_11(self, msg):
        self.transformation_matrix[10] = msg[1]

    def _matrix_cb_12(self, msg):
        self.transformation_matrix[11] = msg[1]

    def _matrix_cb_13(self, msg):
        self.transformation_matrix[12] = msg[1]

    def _matrix_cb_14(self, msg):
        self.transformation_matrix[13] = msg[1]

    def _matrix_cb_15(self, msg):
        self.transformation_matrix[14] = msg[1]
            
    def _matrix_cb_16(self, msg):
        self.transformation_matrix[15] = msg[1]

    def _matrix_cb_17(self, msg):
        self.transformation_matrix[16] = msg[1]

class VrepInterface(object):

## Object Names set
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

    OBJ_NAMES = INDY_OBJ_NAMES + SENSOR_OBJ_NAMES

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

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        # defualt variables
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
        print(self.FOV_distance_sensor.transformation_matrix[0])

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

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

    
if __name__ == '__main__':
    
    vrep = VrepInterface()
    vrep.start_simulation()

    while vrep.flag:
        vrep.step_simulation()

    vrep.stop_simulation()