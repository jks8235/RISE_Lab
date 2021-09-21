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

class VrepDistanceSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_names):
        # variables
        self.client = client
        self.obj_handles = obj_handles
        self.sensor_names = sensor_names
        self.default_distance = 0.8

        self.FOV_distance_1 = [self.default_distance for i in range(19)]
        self.FOV_object_check_1 = [0 for i in range(19)]
        self.sensor_pos_1 = [0 for i in range(19)]

        # create vrep B0 subscriber function
        # Read proximitisensor
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_1, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[1]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_2, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[2]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_3, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[3]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_4, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[4]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_5, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[5]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_6, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[6]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_7, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[7]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_8, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[8]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_9, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[9]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_10, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[10]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_11, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[11]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_12, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[12]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_13, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[13]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_14, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[14]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_15, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[15]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_16, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[16]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_17, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[17]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_18, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[18]], self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._pos_cb_19, dropMessages=True))
    
## Call Back
# Pos Call Back
    def _pos_cb_1(self, msg):
        self.sensor_pos_1[0] = msg[1]

    def _pos_cb_2(self, msg):
        self.sensor_pos_1[1] = msg[1]

    def _pos_cb_3(self, msg):
        self.sensor_pos_1[2] = msg[1]

    def _pos_cb_4(self, msg):
        self.sensor_pos_1[3] = msg[1]

    def _pos_cb_5(self, msg):
        self.sensor_pos_1[4] = msg[1]

    def _pos_cb_6(self, msg):
        self.sensor_pos_1[5] = msg[1]

    def _pos_cb_7(self, msg):
        self.sensor_pos_1[6] = msg[1]

    def _pos_cb_8(self, msg):
        self.sensor_pos_1[7] = msg[1]

    def _pos_cb_9(self, msg):
        self.sensor_pos_1[8] = msg[1]

    def _pos_cb_10(self, msg):
        self.sensor_pos_1[9] = msg[1]

    def _pos_cb_11(self, msg):
        self.sensor_pos_1[10] = msg[1]

    def _pos_cb_12(self, msg):
        self.sensor_pos_1[11] = msg[1]

    def _pos_cb_13(self, msg):
        self.sensor_pos_1[12] = msg[1]

    def _pos_cb_14(self, msg):
        self.sensor_pos_1[13] = msg[1]

    def _pos_cb_15(self, msg):
        self.sensor_pos_1[14] = msg[1]

    def _pos_cb_16(self, msg):
        self.sensor_pos_1[15] = msg[1]

    def _pos_cb_17(self, msg):
        self.sensor_pos_1[16] = msg[1]

    def _pos_cb_18(self, msg):
        self.sensor_pos_1[17] = msg[1]

    def _pos_cb_19(self, msg):
        self.sensor_pos_1[18] = msg[1]

class VrepInterface(object):

    ## Object Names set

    SENSOR_OBJ_NAMES = [
        "Proximity_sensor_1_1",
        "Proximity_sensor_1_2",
        "Proximity_sensor_1_3",
        "Proximity_sensor_1_4",
        "Proximity_sensor_1_5",
        "Proximity_sensor_1_6",
        "Proximity_sensor_2_1",
        "Proximity_sensor_2_2",
        "Proximity_sensor_2_3",
        "Proximity_sensor_2_4",
        "Proximity_sensor_2_5",
        "Proximity_sensor_2_6",
        "Proximity_sensor_3_1",
        "Proximity_sensor_3_2",
        "Proximity_sensor_3_3",
        "Proximity_sensor_3_4",
        "Proximity_sensor_3_5",
        "Proximity_sensor_3_6",
        "Proximity_sensor_Center",
        ]

    OBJ_NAMES = SENSOR_OBJ_NAMES

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
        self.FOV_distance_sensor = VrepDistanceSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        ## data
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

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        for i in range(0,len(self.FOV_distance_sensor.sensor_pos_1)):
            print(i, self.FOV_distance_sensor.sensor_pos_1[i])
        
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

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

if __name__ == '__main__':
    input_data = []
    output_data = []
    extra_data = []

    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    save_count = 1

    while vrep.flag:
        vrep.step_simulation()

    vrep.stop_simulation()