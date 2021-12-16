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
            self.client.simxSetJointPosition(self.obj_handles[name], desired_position, self.joint_state_pub_topics[name])

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

class VrepDistanceSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_names):
        # variables
        self.client = client
        self.obj_handles = obj_handles
        self.sensor_names = sensor_names

        self.collision_check = [0 for i in range(4)]

        self.sensor_pos = []

        # create vrep B0 subscriber function
        '''
        총 5개의 sensor_names가 들어온다.
        0번은 Mount - 전체 Pose를 결정하기 위해 필요하다
        1~4번은 Proximity Sensor이다.

        '''
        # Get mount Pose

        print(self.sensor_names)

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb, dropMessages=True))

        # Read proximitisensor
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[4]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_4, dropMessages=True))
    
    def _ellips_limit(self,point):
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])

        thresh_x = 0.05 # 센서 너비 range
        thresh_y = 0.03 # 센서 높이 range
        thresh_z = 0.1  # 센서 측정 거리

        # print(x, thresh_x, y, thresh_y, z, thresh_z)

        # sensor_range_value = (x/thresh_x)**2 + (y/thresh_y)**2 + (z/thresh_z)**2

        # print(sensor_range_value)
        # print(x, y, z, (x/thresh_x)**2 + (y/thresh_y)**2 + (z/thresh_z)**2)

        if  (x/thresh_x)**2 + (y/thresh_y)**2 + (z/thresh_z)**2 > 1.0:
            check_flag = True

        else:
            check_flag = False

        return check_flag

## Call Back
# Pos Call Back
    def _pos_cb(self, msg):
        self.sensor_pos = msg[1]

# Proxi Sensor Call Back
    def _proxi_sensor_cb_1(self, msg):
        if msg[1] == 1:
            self.collision_check[0] = 1
            if self._ellips_limit(msg[3]) == True:
                self.collision_check[0] = 0
            else:
                self.collision_check[0] = 1
        else:
            self.collision_check[0] = 0

    def _proxi_sensor_cb_2(self, msg):
        if msg[1] == 1:
            self.collision_check[1] = 1
            if self._ellips_limit(msg[3]) == True:
                self.collision_check[1] = 0
            else:
                self.collision_check[1] = 1
        else:
            self.collision_check[1] = 0

    def _proxi_sensor_cb_3(self, msg):
        if msg[1] == 1:
            self.collision_check[2] = 1
            if self._ellips_limit(msg[3]) == True:
                self.collision_check[2] = 0
            else:
                self.collision_check[2] = 1
        else:
            self.collision_check[2] = 0

    def _proxi_sensor_cb_4(self, msg):
        if msg[1] == 1:
            self.collision_check[3] = 1
            if self._ellips_limit(msg[3]) == True:
                self.collision_check[3] = 0
            else:
                self.collision_check[3] = 1
        else:
            self.collision_check[3] = 0

class VrepTotalSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_mounts_names):
        self.sensor_mounts = [0 for i in range(17)]

        for i in range(17):
            self.sensor_mounts[i] = VrepDistanceSensorBridge(client, obj_handles, sensor_mounts_names[i])

class VrepInterface(object):
    def __init__(self):

        ## Object Names set
        self.ROBOT_OBJ_NAMES = [
            "joint_0",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
        ]

        self.SENSOR_MOUNT_OBJ_NAMES = [self.make_sensor_names(i+1) for i in range(17)]

        ### Vrep
        # vrep client variables
        self.do_next_step = True

        # connect to vrep server
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi', 'b0RemoteApi')

        # vrep basic option set
        self.get_object_handles()
        self.client.simxSynchronous(True)

        # vrep bridge define
        self.robot = VrepRobotStateBridge(self.client, self.obj_handles, self.ROBOT_OBJ_NAMES)
        self.total_sensor = VrepTotalSensorBridge(self.client, self.obj_handles, self.SENSOR_MOUNT_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        ## data
        # defualt variables
        self.data = []

        self.flag = True

    def make_sensor_names(self, num):
        sensor_name = [
            "mount_" + str(num),
            "Proximity_sensor_" + str(num) + "_1",
            "Proximity_sensor_" + str(num) + "_2",
            "Proximity_sensor_" + str(num) + "_3",
            "Proximity_sensor_" + str(num) + "_4",
        ]
        return sensor_name

    def get_object_handles(self):
        self.obj_handles = dict()

        OBJ_NAMES = self.ROBOT_OBJ_NAMES
        for i in range(17):
            OBJ_NAMES += self.SENSOR_MOUNT_OBJ_NAMES[i]

        for name in OBJ_NAMES:
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
            self.array_data = np.array(self.data)
            print(self.array_data, self.array_data.shape)
        else:
            self.robot.excute_pose(self.trajectory[0])
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        self.temp_data = []

        for i in range(17):
            if self.total_sensor.sensor_mounts[i].sensor_pos[2] < 0:
                error_flag = True
                print("robot collide the bottom")
                break
            else:
                error_flag = False
                self.temp_data += [self.total_sensor.sensor_mounts[i].sensor_pos + self.total_sensor.sensor_mounts[i].collision_check] # [mount pos x,y,z,wx,wy,wz,w, collision 값 4개] 총 11개

        if error_flag == True:
            self.temp_data = []
        else:
            self.data += [self.temp_data]
            self.temp_data = []

        self.do_next_step = True
        time.sleep(0.001)

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

    def make_path_set(self, resol = 10):

        resolution = resol

        theta_0_range = range(-175, 175+1 ,resolution) # -175 ~ 175
        theta_1_range = range(-100, 100+1 ,resolution) # -175 ~ 175 
        theta_2_range = range(-150, 150+1, resolution) # -175 ~ 175
        theta_3_range = range(-175, 175+1 ,resolution) # -175 ~ 175
        theta_4_range = range(-175, 175+1 ,resolution) # -175 ~ 175
        # theta_5_range = range(-215, 215+1 ,resolution) # -215 ~ 215

        self.trajectory = []

        for theta_0 in theta_0_range:
            for theth_1 in theta_1_range:
                for theth_2 in theta_2_range:
                    for theth_3 in theta_3_range:
                        for theth_4 in theta_4_range:
                            self.trajectory += [[float(theta_0), float(theth_1), float(theth_2), float(theth_3), float(theth_4), 0.0]]

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

if __name__ == '__main__':
    
    vrep = VrepInterface()

    vrep.make_path_set(20)

    data_num = len(vrep.trajectory)
    print(data_num)

    count = 0

    vrep.start_simulation()
        
    while vrep.flag:
        vrep.step_simulation()
        count += 1
        print("%d/%d")%(count, data_num)

    np.save('/media/jee/FC12-B7D8/data_folder/Best_sensor_position/20_resol.npy',vrep.array_data)
  
    # np.save('3dsave.npy',vrep.array_data)

    # input_np_data = pd.DataFrame(np.array(input_data))
    # output_np_data = pd.DataFrame(np.array(output_data))

    # save_data_as_pickle(input_np_data, 'input_Fold_%d'%(save_count))
    # save_data_as_pickle(output_np_data, 'output_Fold_%d'%(save_count))

    # save_data_as_csv(input_np_data, 'input_Fold_%d'%(save_count))
    # save_data_as_csv(output_np_data, 'output_Fold_%d'%(save_count))
#--------------------------------------------------------------------

    vrep.stop_simulation()