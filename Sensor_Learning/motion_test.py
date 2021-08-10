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

    def excute_pose(self, joint_pose):

        pose = joint_to_pose(joint_pose)

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

class VrepInterface(object):

## Object Names set
    MOUNT_OBJ_NAMES = ['ConcretBlock']

    OBJ_NAMES = MOUNT_OBJ_NAMES

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

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

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

    def set_trajectory(self, start_joint_pose, end_joint_pose, step=20):
        self.trajectory = []
        step_joint_pose = [((end_joint_pose[i]-start_joint_pose[i])/step) for i in range(6)]

        for pos_num in range(step+1):
            self.trajectory.append([deg2rad(start_joint_pose[i]+pos_num*step_joint_pose[i]) for i in range(6)])

        self.flag = True

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def make_path_set(resolution):
    theta_0_range = range(-150, 150+1 ,resolution)
    theta_1_range = range(-150, 150+1 ,resolution)
    theta_2_range = range(-150, 150+1, resolution)
    theta_3_range = range(-150, 150+1 ,resolution)
    theta_4_range = range(-150, 150+1 ,resolution)
    theta_5_range = range(-150, 150+1 ,resolution)

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
                                            for theta_5_start in theta_5_range:
                                                for theta_5_end in theta_5_range:
                                                    
                                                    start_point += [[float(theta_0_start), float(theta_1_start), float(theta_2_start), float(theta_3_start), float(theta_4_start), float(theta_5_start)]]
                                                    end_point += [[float(theta_0_end), float(theta_1_end), float(theta_2_end), float(theta_3_end), float(theta_4_end), float(theta_5_end)]]

    angle_path = zip(start_point, end_point)

    return angle_path

if __name__ == '__main__':
    angle_path = make_path_set(180)

    print(len(angle_path))
    
    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0

    for start_angle, end_angle in angle_path:

        count += 1
        print(count)

        vrep.set_trajectory(start_angle, end_angle, step=30)
        
        while vrep.flag:
            vrep.step_simulation()

    vrep.stop_simulation()
