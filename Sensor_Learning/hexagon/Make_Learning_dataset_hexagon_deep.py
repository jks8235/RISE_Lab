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

        self.FOV_distance = [self.default_distance for i in range(37)]
        self.FOV_object_check = [0 for i in range(37)]
        self.sensor_pos = []

        # create vrep B0 subscriber function
        # Read proximitisensor
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_0, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[4]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_4, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[5]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_5, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[6]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_6, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[7]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_7, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[8]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_8, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[9]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_9, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[10]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_10, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[11]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_11, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[12]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_12, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[13]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_13, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[14]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_14, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[15]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_15, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[16]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_16, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[17]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_17, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_18, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[19]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_19, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[20]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_20, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[21]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_21, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[22]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_22, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[23]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_23, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[24]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_24, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[25]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_25, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[26]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_26, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[27]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_27, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[28]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_28, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[29]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_29, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[30]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_30, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[31]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_31, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[32]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_32, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[33]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_33, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[34]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_34, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[35]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_35, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[36]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_36, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb, dropMessages=True))

    def get_distance(self):
    
        AVG_distance = sum(self.FOV_distance)/len(self.FOV_distance)

        return AVG_distance
    
## Call Back
# Pos Call Back
    def _pos_cb(self, msg):
        self.sensor_pos = msg[1]

# Proxi Sensor Call Back 1
    def _proxi_sensor_cb_0(self, msg):
        if msg[1] == 1:
            self.FOV_distance[0] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[0] = self.default_distance
            self.FOV_object_check[0] = 0

    def _proxi_sensor_cb_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance[1] = msg[2]
            self.FOV_object_check[1] = 1
        else:
            self.FOV_distance[1] = self.default_distance
            self.FOV_object_check[1] = 0

    def _proxi_sensor_cb_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance[2] = msg[2]
            self.FOV_object_check[2] = 1
        else:
            self.FOV_distance[2] = self.default_distance
            self.FOV_object_check[2] = 0

    def _proxi_sensor_cb_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance[3] = msg[2]
            self.FOV_object_check[3] = 1
        else:
            self.FOV_distance[3] = self.default_distance
            self.FOV_object_check[3] = 0

    def _proxi_sensor_cb_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance[4] = msg[2]
            self.FOV_object_check[4] = 1
        else:
            self.FOV_distance[4] = self.default_distance
            self.FOV_object_check[4] = 0

    def _proxi_sensor_cb_5(self, msg):
        if msg[1] == 1:
            self.FOV_distance[5] = msg[2]
            self.FOV_object_check[5] = 1
        else:
            self.FOV_distance[5] = self.default_distance
            self.FOV_object_check[5] = 0

    def _proxi_sensor_cb_6(self, msg):
        if msg[1] == 1:
            self.FOV_distance[6] = msg[2]
            self.FOV_object_check[6] = 1
        else:
            self.FOV_distance[6] = self.default_distance
            self.FOV_object_check[6] = 0

    def _proxi_sensor_cb_7(self, msg):
        if msg[1] == 1:
            self.FOV_distance[7] = msg[2]
            self.FOV_object_check[7] = 1
        else:
            self.FOV_distance[7] = self.default_distance
            self.FOV_object_check[7] = 0

    def _proxi_sensor_cb_8(self, msg):
        if msg[1] == 1:
            self.FOV_distance[8] = msg[2]
            self.FOV_object_check[8] = 1
        else:
            self.FOV_distance[8] = self.default_distance
            self.FOV_object_check[8] = 0

    def _proxi_sensor_cb_9(self, msg):
        if msg[1] == 1:
            self.FOV_distance[9] = msg[2]
            self.FOV_object_check[9] = 1
        else:
            self.FOV_distance[9] = self.default_distance
            self.FOV_object_check[9] = 0

    def _proxi_sensor_cb_10(self, msg):
        if msg[1] == 1:
            self.FOV_distance[10] = msg[2]
            self.FOV_object_check[10] = 1
        else:
            self.FOV_distance[10] = self.default_distance
            self.FOV_object_check[10] = 0

    def _proxi_sensor_cb_11(self, msg):
        if msg[1] == 1:
            self.FOV_distance[11] = msg[2]
            self.FOV_object_check[11] = 1
        else:
            self.FOV_distance[11] = self.default_distance
            self.FOV_object_check[11] = 0

    def _proxi_sensor_cb_12(self, msg):
        if msg[1] == 1:
            self.FOV_distance[12] = msg[2]
            self.FOV_object_check[12] = 1
        else:
            self.FOV_distance[12] = self.default_distance
            self.FOV_object_check[12] = 0

    def _proxi_sensor_cb_13(self, msg):
        if msg[1] == 1:
            self.FOV_distance[13] = msg[2]
            self.FOV_object_check[13] = 1
        else:
            self.FOV_distance[13] = self.default_distance
            self.FOV_object_check[13] = 0

    def _proxi_sensor_cb_14(self, msg):
        if msg[1] == 1:
            self.FOV_distance[14] = msg[2]
            self.FOV_object_check[14] = 1
        else:
            self.FOV_distance[14] = self.default_distance
            self.FOV_object_check[14] = 0

    def _proxi_sensor_cb_15(self, msg):
        if msg[1] == 1:
            self.FOV_distance[15] = msg[2]
            self.FOV_object_check[15] = 1
        else:
            self.FOV_distance[15] = self.default_distance
            self.FOV_object_check[15] = 0

    def _proxi_sensor_cb_16(self, msg):
        if msg[1] == 1:
            self.FOV_distance[16] = msg[2]
            self.FOV_object_check[16] = 1
        else:
            self.FOV_distance[16] = self.default_distance
            self.FOV_object_check[16] = 0

    def _proxi_sensor_cb_17(self, msg):
        if msg[1] == 1:
            self.FOV_distance[17] = msg[2]
            self.FOV_object_check[17] = 1
        else:
            self.FOV_distance[17] = self.default_distance
            self.FOV_object_check[17] = 0

    def _proxi_sensor_cb_18(self, msg):
        if msg[1] == 1:
            self.FOV_distance[18] = msg[2]
            self.FOV_object_check[18] = 1
        else:
            self.FOV_distance[18] = self.default_distance
            self.FOV_object_check[18] = 0

    def _proxi_sensor_cb_19(self, msg):
        if msg[1] == 1:
            self.FOV_distance[19] = msg[2]
            self.FOV_object_check[19] = 1
        else:
            self.FOV_distance[19] = self.default_distance
            self.FOV_object_check[19] = 0

    def _proxi_sensor_cb_20(self, msg):
        if msg[1] == 1:
            self.FOV_distance[20] = msg[2]
            self.FOV_object_check[20] = 1
        else:
            self.FOV_distance[20] = self.default_distance
            self.FOV_object_check[20] = 0

    def _proxi_sensor_cb_21(self, msg):
        if msg[1] == 1:
            self.FOV_distance[21] = msg[2]
            self.FOV_object_check[21] = 1
        else:
            self.FOV_distance[21] = self.default_distance
            self.FOV_object_check[21] = 0

    def _proxi_sensor_cb_22(self, msg):
        if msg[1] == 1:
            self.FOV_distance[22] = msg[2]
            self.FOV_object_check[22] = 1
        else:
            self.FOV_distance[22] = self.default_distance
            self.FOV_object_check[22] = 0

    def _proxi_sensor_cb_23(self, msg):
        if msg[1] == 1:
            self.FOV_distance[23] = msg[2]
            self.FOV_object_check[23] = 1
        else:
            self.FOV_distance[23] = self.default_distance
            self.FOV_object_check[23] = 0

    def _proxi_sensor_cb_24(self, msg):
        if msg[1] == 1:
            self.FOV_distance[24] = msg[2]
            self.FOV_object_check[24] = 1
        else:
            self.FOV_distance[24] = self.default_distance
            self.FOV_object_check[24] = 0

    def _proxi_sensor_cb_25(self, msg):
        if msg[1] == 1:
            self.FOV_distance[25] = msg[2]
            self.FOV_object_check[25] = 1
        else:
            self.FOV_distance[25] = self.default_distance
            self.FOV_object_check[25] = 0

    def _proxi_sensor_cb_26(self, msg):
        if msg[1] == 1:
            self.FOV_distance[26] = msg[2]
            self.FOV_object_check[26] = 1
        else:
            self.FOV_distance[26] = self.default_distance
            self.FOV_object_check[26] = 0

    def _proxi_sensor_cb_27(self, msg):
        if msg[1] == 1:
            self.FOV_distance[27] = msg[2]
            self.FOV_object_check[27] = 1
        else:
            self.FOV_distance[27] = self.default_distance
            self.FOV_object_check[27] = 0

    def _proxi_sensor_cb_28(self, msg):
        if msg[1] == 1:
            self.FOV_distance[28] = msg[2]
            self.FOV_object_check[28] = 1
        else:
            self.FOV_distance[28] = self.default_distance
            self.FOV_object_check[28] = 0

    def _proxi_sensor_cb_29(self, msg):
        if msg[1] == 1:
            self.FOV_distance[29] = msg[2]
            self.FOV_object_check[29] = 1
        else:
            self.FOV_distance[29] = self.default_distance
            self.FOV_object_check[29] = 0

    def _proxi_sensor_cb_30(self, msg):
        if msg[1] == 1:
            self.FOV_distance[30] = msg[2]
            self.FOV_object_check[30] = 1
        else:
            self.FOV_distance[30] = self.default_distance
            self.FOV_object_check[30] = 0

    def _proxi_sensor_cb_31(self, msg):
        if msg[1] == 1:
            self.FOV_distance[31] = msg[2]
            self.FOV_object_check[31] = 1
        else:
            self.FOV_distance[31] = self.default_distance
            self.FOV_object_check[31] = 0

    def _proxi_sensor_cb_32(self, msg):
        if msg[1] == 1:
            self.FOV_distance[32] = msg[2]
            self.FOV_object_check[32] = 1
        else:
            self.FOV_distance[32] = self.default_distance
            self.FOV_object_check[32] = 0

    def _proxi_sensor_cb_33(self, msg):
        if msg[1] == 1:
            self.FOV_distance[33] = msg[2]
            self.FOV_object_check[33] = 1
        else:
            self.FOV_distance[33] = self.default_distance
            self.FOV_object_check[33] = 0

    def _proxi_sensor_cb_34(self, msg):
        if msg[1] == 1:
            self.FOV_distance[34] = msg[2]
            self.FOV_object_check[34] = 1
        else:
            self.FOV_distance[34] = self.default_distance
            self.FOV_object_check[34] = 0

    def _proxi_sensor_cb_35(self, msg):
        if msg[1] == 1:
            self.FOV_distance[35] = msg[2]
            self.FOV_object_check[35] = 1
        else:
            self.FOV_distance[35] = self.default_distance
            self.FOV_object_check[35] = 0

    def _proxi_sensor_cb_36(self, msg):
        if msg[1] == 1:
            self.FOV_distance[36] = msg[2]
            self.FOV_object_check[36] = 1
        else:
            self.FOV_distance[36] = self.default_distance
            self.FOV_object_check[36] = 0

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
        "Proximity_sensor_3_01",
        "Proximity_sensor_3_02",
        "Proximity_sensor_3_03",
        "Proximity_sensor_3_04",
        "Proximity_sensor_3_05",
        "Proximity_sensor_3_06",
        "Proximity_sensor_3_07",
        "Proximity_sensor_3_08",
        "Proximity_sensor_3_09",
        "Proximity_sensor_3_10",
        "Proximity_sensor_3_11",
        "Proximity_sensor_3_12",
        "Proximity_sensor_3_13",
        "Proximity_sensor_3_14",
        "Proximity_sensor_3_15",
        "Proximity_sensor_3_16",
        "Proximity_sensor_3_17",
        "Proximity_sensor_3_18",
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
            self.indy.excute_pose(self.trajectory[0])
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        FOV_distance = self.FOV_distance_sensor.FOV_distance
        object_check = self.FOV_distance_sensor.FOV_object_check
        bundle_pos = self.FOV_distance_sensor.sensor_pos

        AVG_distance = self.FOV_distance_sensor.get_distance()

        if self.flag == False:
        
            # add data set
            self.input_data = self.input_temp
            self.output_data = self.output_temp

            self.make_data()

            # clear temp data
            self.input_temp = []
            self.output_temp = []

        else:
            self.input_temp = self.input_temp + [bundle_pos + [AVG_distance]]
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

def save_data_as_csv(data, name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_deep/test_2/' + name + '.csv'
    data.to_csv(path, sep=',', header=None, index=None)

def save_data_as_pickle(data, name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_deep/test_2/' + name + '.pkl'
    data.to_pickle(path)

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

    angle_path = make_path_set()
    obj_poses = make_object_pos_set(0.25)

    print(len(angle_path), len(obj_poses))
    data_num = (len(angle_path)*len(obj_poses))
    print(data_num)
    
    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    save_count = 1

#---------------------------------------------------------
    # # learning data
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

    #     input_np_data = pd.DataFrame(np.array(input_data))
    #     output_np_data = pd.DataFrame(np.array(output_data))

    #     save_data_as_pickle(input_np_data, 'input_Fold_%d'%(save_count))
    #     save_data_as_pickle(output_np_data, 'output_Fold_%d'%(save_count))

    #     # save_data_as_csv(input_np_data, 'input_Fold_%d'%(save_count))
    #     # save_data_as_csv(output_np_data, 'output_Fold_%d'%(save_count))

    #     input_data = []
    #     output_data = []

    #     save_count += 1
#----------------------------------------------------------------------
    # Test data
    for start_angle, end_angle in angle_path:

        vrep.set_trajectory(start_angle, end_angle, step=350)
        
        while vrep.flag:
            vrep.step_simulation()

        input_data += copy.deepcopy(vrep.learning_input)
        output_data += copy.deepcopy(vrep.learning_output)

        count += 1
        print("%d/%d")%(count, data_num)

    input_np_data = pd.DataFrame(np.array(input_data))
    output_np_data = pd.DataFrame(np.array(output_data))

    save_data_as_pickle(input_np_data, 'input_Fold_%d'%(save_count))
    save_data_as_pickle(output_np_data, 'output_Fold_%d'%(save_count))

    # save_data_as_csv(input_np_data, 'input_Fold_%d'%(save_count))
    # save_data_as_csv(output_np_data, 'output_Fold_%d'%(save_count))
#--------------------------------------------------------------------

    vrep.stop_simulation()

    print(input_np_data.shape)
    print(output_np_data.shape)