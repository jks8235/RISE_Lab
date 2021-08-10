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
from scipy.spatial.transform import Rotation as R

sys.path.insert(0 , '/home/jee/work_space/catkin_wk/src/RISE_Lab/Library/')
from CoppeliaSim_bluezero import b0RemoteApi


class VrepMountStateBridge(object):
    def __init__(self, client, obj_handles, mount_names):
        self.client = client
        self.obj_handles = obj_handles
        self.mount_name = mount_names

        # create pub topics
        self.mount_state_pub_topics = dict()
        for name in mount_names:
            self.mount_state_pub_topics[name] = self.client.simxCreatePublisher(True)

    def excute_pose(self, pose):

        self.client.simxSetObjectPose(self.obj_handles[self.mount_name[0]], -1, pose, self.mount_state_pub_topics[self.mount_name[0]])

def joint_to_pose(joint_pose):

    L1_1 = 0.2
    L1_2 = 0.2
    L2   = 0.4
    L3_1 = 0.2
    L3_2 = 0.3
    L4_1 = 0.15
    L4_2 = 0.2
    L5_1 = 0.2
    L5_2 = 0.2
    L6   = 0.1

    Tran_mat_1_1 = DH_to_TransMatrix(joint_pose[0], L1_1, L1_2, 0.0)
    Tran_mat_1_2 = DH_to_TransMatrix(deg2rad(-90), 0.0, 0.0, deg2rad(-90))
    Tran_mat_2   = DH_to_TransMatrix(joint_pose[1]-deg2rad(90), 0.0, L2, deg2rad(180))
    Tran_mat_3_1 = DH_to_TransMatrix(joint_pose[2], L3_1, L3_2, 0.0)
    Tran_mat_3_2 = DH_to_TransMatrix(deg2rad(-90), 0.0, 0.0, deg2rad(-90))
    Tran_mat_4_1 = DH_to_TransMatrix(joint_pose[3], L4_1, L4_2, 0.0)
    Tran_mat_4_2 = DH_to_TransMatrix(deg2rad(-90), 0.0, 0.0, deg2rad(-90))
    Tran_mat_5_1 = DH_to_TransMatrix(joint_pose[4], L5_1, L5_2, 0.0)
    Tran_mat_5_2 = DH_to_TransMatrix(deg2rad(90), 0.0, 0.0, deg2rad(90))
    Tran_mat_6   = DH_to_TransMatrix(joint_pose[5], L6, 0.0, 0.0)

    Tran_mat_list = [Tran_mat_1_1, Tran_mat_1_2, Tran_mat_2, Tran_mat_3_1, Tran_mat_3_2, Tran_mat_4_1, Tran_mat_4_2, Tran_mat_5_1, Tran_mat_5_2, Tran_mat_6]
    total_mat = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]])

    for mat in Tran_mat_list:
        total_mat = np.dot(total_mat,mat)

    rotation_mat = total_mat[:3,:3]

    quarternion = R.as_quat(R.from_dcm(rotation_mat)).tolist()

    translation = [total_mat[0,3], total_mat[1,3], total_mat[2,3]]

    pose = translation + quarternion

    return pose

def DH_to_TransMatrix(theta, d, a, alpha):
    Trans_matrix = np.array([[math.cos(theta),-math.sin(theta)*math.cos(alpha),math.sin(theta)*math.sin(alpha) ,a*math.cos(theta)],
                             [math.sin(theta),math.cos(theta)*math.cos(alpha) ,-math.cos(theta)*math.sin(alpha),a*math.sin(theta)],
                             [0.0            ,math.sin(alpha)                 ,math.cos(alpha)                 ,d                ],
                             [0.0            ,0.0                             ,0.0                             ,1.0              ]])

    return Trans_matrix

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

        self.FOV_distance_1 = [self.default_distance for i in range(19)]
        self.FOV_distance_2 = [self.default_distance for i in range(19)]
        self.FOV_object_check_1 = [0 for i in range(19)]
        self.FOV_object_check_2 = [0 for i in range(19)]
        self.sensor_pos_1 = []
        self.sensor_pos_2 = []

        # create vrep B0 subscriber function
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_4, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[4]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_5, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[5]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_6, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[6]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_7, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[7]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_8, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[8]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_9, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[9]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_10, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[10]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_11, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[11]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_12, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[12]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_13, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[13]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_14, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[14]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_15, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[15]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_16, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[16]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_17, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[17]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_18, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._proxi_sensor_1_cb_19, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb_1, dropMessages=True))


        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[19]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[20]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[21]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[22]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_4, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[23]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_5, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[24]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_6, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[25]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_7, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[26]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_8, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[27]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_9, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[28]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_10, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[29]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_11, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[30]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_12, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[31]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_13, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[32]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_14, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[33]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_15, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[34]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_16, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[35]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_17, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[36]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_18, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[37]], self.client.simxCreateSubscriber(self._proxi_sensor_2_cb_19, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[19]], -1, self.client.simxCreateSubscriber(self._pos_cb_2, dropMessages=True))

    def get_distance(self):
    
        AVG_distance_1 = sum(self.FOV_distance_1)/len(self.FOV_distance_1)
        AVG_distance_2 = sum(self.FOV_distance_2)/len(self.FOV_distance_2)

        return AVG_distance_1, AVG_distance_2
    
## Call Back
# Pos Call Back
    def _pos_cb_1(self, msg):
        self.sensor_pos_1 = msg[1]

    def _pos_cb_2(self, msg):
        self.sensor_pos_2 = msg[1]

# Proxi Sensor Call Back 1
    def _proxi_sensor_1_cb_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[0] = msg[2]
            self.FOV_object_check_1[0] = 1
        else:
            self.FOV_distance_1[0] = self.default_distance
            self.FOV_object_check_1[0] = 0
            
    def _proxi_sensor_1_cb_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[1] = msg[2]
            self.FOV_object_check_1[1] = 1
        else:
            self.FOV_distance_1[1] = self.default_distance
            self.FOV_object_check_1[1] = 0
            
    def _proxi_sensor_1_cb_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[2] = msg[2]
            self.FOV_object_check_1[2] = 1
        else:
            self.FOV_distance_1[2] = self.default_distance
            self.FOV_object_check_1[2] = 0
            
    def _proxi_sensor_1_cb_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[3] = msg[2]
            self.FOV_object_check_1[3] = 1
        else:
            self.FOV_distance_1[3] = self.default_distance
            self.FOV_object_check_1[3] = 0
            
    def _proxi_sensor_1_cb_5(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[4] = msg[2]
            self.FOV_object_check_1[4] = 1
        else:
            self.FOV_distance_1[4] = self.default_distance
            self.FOV_object_check_1[4] = 0
            
    def _proxi_sensor_1_cb_6(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[5] = msg[2]
            self.FOV_object_check_1[5] = 1
        else:
            self.FOV_distance_1[5] = self.default_distance
            self.FOV_object_check_1[5] = 0
            
    def _proxi_sensor_1_cb_7(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[6] = msg[2]
            self.FOV_object_check_1[6] = 1
        else:
            self.FOV_distance_1[6] = self.default_distance
            self.FOV_object_check_1[6] = 0
            
    def _proxi_sensor_1_cb_8(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[7] = msg[2]
            self.FOV_object_check_1[7] = 1
        else:
            self.FOV_distance_1[7] = self.default_distance
            self.FOV_object_check_1[7] = 0
            
    def _proxi_sensor_1_cb_9(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[8] = msg[2]
            self.FOV_object_check_1[8] = 1
        else:
            self.FOV_distance_1[8] = self.default_distance
            self.FOV_object_check_1[8] = 0
            
    def _proxi_sensor_1_cb_10(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[9] = msg[2]
            self.FOV_object_check_1[9] = 1
        else:
            self.FOV_distance_1[9] = self.default_distance
            self.FOV_object_check_1[9] = 0
            
    def _proxi_sensor_1_cb_11(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[10] = msg[2]
            self.FOV_object_check_1[10] = 1
        else:
            self.FOV_distance_1[10] = self.default_distance
            self.FOV_object_check_1[10] = 0
            
    def _proxi_sensor_1_cb_12(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[11] = msg[2]
            self.FOV_object_check_1[11] = 1
        else:
            self.FOV_distance_1[11] = self.default_distance
            self.FOV_object_check_1[11] = 0
            
    def _proxi_sensor_1_cb_13(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[12] = msg[2]
            self.FOV_object_check_1[12] = 1
        else:
            self.FOV_distance_1[12] = self.default_distance
            self.FOV_object_check_1[12] = 0
            
    def _proxi_sensor_1_cb_14(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[13] = msg[2]
            self.FOV_object_check_1[13] = 1
        else:
            self.FOV_distance_1[13] = self.default_distance
            self.FOV_object_check_1[13] = 0
            
    def _proxi_sensor_1_cb_15(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[14] = msg[2]
            self.FOV_object_check_1[14] = 1
        else:
            self.FOV_distance_1[14] = self.default_distance
            self.FOV_object_check_1[14] = 0
            
    def _proxi_sensor_1_cb_16(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[15] = msg[2]
            self.FOV_object_check_1[15] = 1
        else:
            self.FOV_distance_1[15] = self.default_distance
            self.FOV_object_check_1[15] = 0
            
    def _proxi_sensor_1_cb_17(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[16] = msg[2]
            self.FOV_object_check_1[16] = 1
        else:
            self.FOV_distance_1[16] = self.default_distance
            self.FOV_object_check_1[16] = 0

    def _proxi_sensor_1_cb_18(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[17] = msg[2]
            self.FOV_object_check_1[17] = 1
        else:
            self.FOV_distance_1[17] = self.default_distance
            self.FOV_object_check_1[17] = 0

    def _proxi_sensor_1_cb_19(self, msg):
        if msg[1] == 1:
            self.FOV_distance_1[18] = msg[2]
            self.FOV_object_check_1[18] = 1
        else:
            self.FOV_distance_1[18] = self.default_distance
            self.FOV_object_check_1[18] = 0

# Proxi Sensor Call Back 2
    def _proxi_sensor_2_cb_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[0] = msg[2]
            self.FOV_object_check_2[0] = 1
        else:
            self.FOV_distance_2[0] = self.default_distance
            self.FOV_object_check_2[0] = 0
            
    def _proxi_sensor_2_cb_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[1] = msg[2]
            self.FOV_object_check_2[1] = 1
        else:
            self.FOV_distance_2[1] = self.default_distance
            self.FOV_object_check_2[1] = 0
            
    def _proxi_sensor_2_cb_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[2] = msg[2]
            self.FOV_object_check_2[2] = 1
        else:
            self.FOV_distance_2[2] = self.default_distance
            self.FOV_object_check_2[2] = 0
            
    def _proxi_sensor_2_cb_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[3] = msg[2]
            self.FOV_object_check_2[3] = 1
        else:
            self.FOV_distance_2[3] = self.default_distance
            self.FOV_object_check_2[3] = 0
            
    def _proxi_sensor_2_cb_5(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[4] = msg[2]
            self.FOV_object_check_2[4] = 1
        else:
            self.FOV_distance_2[4] = self.default_distance
            self.FOV_object_check_2[4] = 0
            
    def _proxi_sensor_2_cb_6(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[5] = msg[2]
            self.FOV_object_check_2[5] = 1
        else:
            self.FOV_distance_2[5] = self.default_distance
            self.FOV_object_check_2[5] = 0
            
    def _proxi_sensor_2_cb_7(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[6] = msg[2]
            self.FOV_object_check_2[6] = 1
        else:
            self.FOV_distance_2[6] = self.default_distance
            self.FOV_object_check_2[6] = 0
            
    def _proxi_sensor_2_cb_8(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[7] = msg[2]
            self.FOV_object_check_2[7] = 1
        else:
            self.FOV_distance_2[7] = self.default_distance
            self.FOV_object_check_2[7] = 0
            
    def _proxi_sensor_2_cb_9(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[8] = msg[2]
            self.FOV_object_check_2[8] = 1
        else:
            self.FOV_distance_2[8] = self.default_distance
            self.FOV_object_check_2[8] = 0
            
    def _proxi_sensor_2_cb_10(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[9] = msg[2]
            self.FOV_object_check_2[9] = 1
        else:
            self.FOV_distance_2[9] = self.default_distance
            self.FOV_object_check_2[9] = 0
            
    def _proxi_sensor_2_cb_11(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[10] = msg[2]
            self.FOV_object_check_2[10] = 1
        else:
            self.FOV_distance_2[10] = self.default_distance
            self.FOV_object_check_2[10] = 0
            
    def _proxi_sensor_2_cb_12(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[11] = msg[2]
            self.FOV_object_check_2[11] = 1
        else:
            self.FOV_distance_2[11] = self.default_distance
            self.FOV_object_check_2[11] = 0
            
    def _proxi_sensor_2_cb_13(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[12] = msg[2]
            self.FOV_object_check_2[12] = 1
        else:
            self.FOV_distance_2[12] = self.default_distance
            self.FOV_object_check_2[12] = 0
            
    def _proxi_sensor_2_cb_14(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[13] = msg[2]
            self.FOV_object_check_2[13] = 1
        else:
            self.FOV_distance_2[13] = self.default_distance
            self.FOV_object_check_2[13] = 0
            
    def _proxi_sensor_2_cb_15(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[14] = msg[2]
            self.FOV_object_check_2[14] = 1
        else:
            self.FOV_distance_2[14] = self.default_distance
            self.FOV_object_check_2[14] = 0
            
    def _proxi_sensor_2_cb_16(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[15] = msg[2]
            self.FOV_object_check_2[15] = 1
        else:
            self.FOV_distance_2[15] = self.default_distance
            self.FOV_object_check_2[15] = 0
            
    def _proxi_sensor_2_cb_17(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[16] = msg[2]
            self.FOV_object_check_2[16] = 1
        else:
            self.FOV_distance_2[16] = self.default_distance
            self.FOV_object_check_2[16] = 0

    def _proxi_sensor_2_cb_18(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[17] = msg[2]
            self.FOV_object_check_2[17] = 1
        else:
            self.FOV_distance_2[17] = self.default_distance
            self.FOV_object_check_2[17] = 0

    def _proxi_sensor_2_cb_19(self, msg):
        if msg[1] == 1:
            self.FOV_distance_2[18] = msg[2]
            self.FOV_object_check_2[18] = 1
        else:
            self.FOV_distance_2[18] = self.default_distance
            self.FOV_object_check_2[18] = 0
            

class VrepInterface(object):

## Object Names set
    BOX_OBJ_NAMES = [
        "Box1",
        "Box2",
        "Box3"
        ]

    MOUNT_OBJ_NAMES = [
        'Mount'
        ]

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
        "Proximity_sensor_1_1#0",
        "Proximity_sensor_1_2#0",
        "Proximity_sensor_1_3#0",
        "Proximity_sensor_1_4#0",
        "Proximity_sensor_1_5#0",
        "Proximity_sensor_1_6#0",
        "Proximity_sensor_2_1#0",
        "Proximity_sensor_2_2#0",
        "Proximity_sensor_2_3#0",
        "Proximity_sensor_2_4#0",
        "Proximity_sensor_2_5#0",
        "Proximity_sensor_2_6#0",
        "Proximity_sensor_3_1#0",
        "Proximity_sensor_3_2#0",
        "Proximity_sensor_3_3#0",
        "Proximity_sensor_3_4#0",
        "Proximity_sensor_3_5#0",
        "Proximity_sensor_3_6#0",
        "Proximity_sensor_Center#0",
        ]

    OBJ_NAMES = BOX_OBJ_NAMES + MOUNT_OBJ_NAMES + SENSOR_OBJ_NAMES

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
        self.mount = VrepMountStateBridge(self.client, self.obj_handles, self.MOUNT_OBJ_NAMES)
        self.FOV_distance_sensor = VrepDistanceSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)
        self.Objects = VrepObjectStateBridge(self.client, self.obj_handles, self.BOX_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        ## data
        # defualt variables
        # self.Total_Data = dict{'sensor_1_center':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_1_1':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_1_2':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_1_3':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_1_4':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_1_5':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_1_6':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_2_1':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_2_2':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_2_3':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_2_4':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_2_5':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_2_6':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_3_1':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_3_2':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_3_3':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_3_4':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_3_5':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_1_3_6':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_center':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_1_1':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_1_2':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_1_3':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_1_4':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_1_5':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_1_6':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_2_1':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_2_2':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_2_3':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_2_4':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_2_5':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_2_6':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_3_1':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_3_2':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_3_3':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_3_4':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_3_5':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]},
        #                        'sensor_2_3_6':{'x':[], 'y':[], 'z':[], 'q_x':[], 'q_y':[], 'q_z':[], 'q_w':[], 'd':[], 'check_bit':[]}}
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
        if (len(self.trajectory) == 0) or (self.pose_error == True):
            self.flag = False
        else:
            self.mount.excute_pose(self.trajectory[0])
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        FOV_distance_1 = self.FOV_distance_sensor.FOV_distance_1
        FOV_distance_2 = self.FOV_distance_sensor.FOV_distance_2
        object_check_1 = self.FOV_distance_sensor.FOV_object_check_1
        object_check_2 = self.FOV_distance_sensor.FOV_object_check_2
        bundle_pos_1 = self.FOV_distance_sensor.sensor_pos_1
        bundle_pos_2 = self.FOV_distance_sensor.sensor_pos_2

        AVG_distance_1, AVG_distance_2 = self.FOV_distance_sensor.get_distance()

        if self.flag == False:
            # self.output_temp = FOV_distance
            if self.pose_error == True:
                pass
            else:
                # add data set
                self.input_data = self.input_temp
                self.output_data = self.output_temp

                self.make_data()

                # clear temp data
                self.input_temp = []
                self.output_temp = []

        else:
            self.input_temp = self.input_temp + [bundle_pos_1 + [AVG_distance_1] + bundle_pos_2 + [AVG_distance_2]]
            self.output_temp = self.output_temp + [FOV_distance_1 + object_check_1 + FOV_distance_2 + object_check_2]
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
        self.pose_error = False
        self.trajectory = []
        joint_pose = []
        step_pose = [((end_pose[i]-start_pose[i])/step) for i in range(6)]

        for pos_num in range(step+1):
            joint_pose.append([deg2rad(start_pose[i]+pos_num*step_pose[i]) for i in range(6)])
        
        for joint_pos in joint_pose:
             self.trajectory.append(joint_to_pose(joint_pos))
             
             if joint_pos[2] < 0.0:
                 self.pose_error = True
                 break

        self.flag = True

    def set_object_position(self, obj_pos):
        self.Objects.excute_pose(obj_pos)

    def make_data(self):

        step = 30
        data_step = 21

        self.learning_input = []
        self.learning_output =[]

        for i in range(len(self.input_data)-20):
            temp_input = self.input_data[i:(i+21)]
            temp_output = self.output_data[(i+20)]

            # print(temp_output)

            self.learning_input += [sum(temp_input, [])]
            self.learning_output += [temp_output]

def _save_data_as_csv(data, name):
    path = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/hexagon/learning_1/' + name + '.csv'
    data.to_csv(path, sep=',', header=None, index=None)

def _save_data_as_pickle(data, name):
    path = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/hexagon/learning_1/' + name + '.csv'
    data.to_pickle(path)

def make_fold(pd_data, fold_num):

    DataNo = pd_data.shape[1]
    input_FeatNo = 336
    output_FeatNo = 76 + input_FeatNo
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
    theta_0_range = range(-175, 175+1 ,resolution)
    theta_1_range = range(-175, 175+1 ,resolution)
    theta_2_range = range(-175, 175+1, resolution)
    theta_3_range = range(-175, 175+1 ,resolution)
    theta_4_range = range(-175, 175+1 ,resolution)
    theta_5_range = range(-215, 215+1 ,resolution)

    start_point = []
    end_point = []

    for theta_0_start in theta_0_range:
        for theta_0_end in theta_0_range:
            for theta_1_start in theta_1_range:
                for theta_1_end in theta_1_range:
                    for theta_2_start in theta_2_range:
                        for theta_2_end in theta_2_range:
                            for theta_3_start in theta_3_range:
                                for theta_3_end in theta_3_range:
                                    for theta_4_start in theta_4_range:
                                        for theta_4_end in theta_4_range:
                                                    
                                            start_point += [[float(theta_0_start), float(theta_1_start), float(theta_2_start), float(theta_3_start), float(theta_4_start), 0.0]]
                                            end_point += [[float(theta_0_end), float(theta_1_end), float(theta_2_end), float(theta_3_end), float(theta_4_end), 0.0]]

    angle_path = zip(start_point, end_point)

    return angle_path

def make_object_pos_set(resolution):
    # input unit (m), calculation unit (mm), output unit (m)
    resolution = int(resolution*1000)

    # situation
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

    angle_path = make_path_set(150)
    obj_poses = make_object_pos_set(0.4)

    print(len(angle_path), len(obj_poses))
    data_num = (len(angle_path)*len(obj_poses))
    print(data_num)
    
    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    error_count = 0

    for obj_pos in obj_poses:
        for start_angle, end_angle in angle_path:

            vrep.set_object_position(obj_pos)
            vrep.set_trajectory(start_angle, end_angle, step=30)
            
            while vrep.flag:
                vrep.step_simulation()

            if vrep.pose_error == True:
                error_count += 1
                print("error", error_count)
            else:
                input_data += copy.deepcopy(vrep.learning_input)
                output_data += copy.deepcopy(vrep.learning_output)

                count += 1
                print(count)

    vrep.stop_simulation()

    # print(len(input_data))
    # print(len(input_data[0]))
    # print(input_data[0])

    # print(len(output_data))
    # print(len(output_data[0]))
    # print(output_data[0])

    input_np_data = np.array(input_data)
    output_np_data = np.array(output_data)

    # input_np_data = np.reshape(input_data,(data_num,168))
    # output_np_data = np.reshape(output_data,(data_num,34))

    print(input_np_data.shape)
    print(output_np_data.shape)

    Total_data = np.concatenate([input_np_data, output_np_data], axis=1)
    pd_Total = pd.DataFrame(Total_data).T

    print(pd_Total.shape)
    print(pd_Total)

    make_fold(pd_Total,10)
