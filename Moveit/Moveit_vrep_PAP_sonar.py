#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import rospy
import time
import pdb
import numpy as np
import sys
import pandas as pd
# import rospy
import math
import copy
import random

from moveit_msgs.msg import DisplayTrajectory

sys.path.insert(0 , '/home/jee/work_space/catkin_wk/src/RISE_Lab/Library/')
from CoppeliaSim_bluezero import b0RemoteApi



# brief
# The progran is simulation about a robot indy7
# The program connects moveit with vrep B0
# Get information about each joint value from movegroup / display_planed_path of moveit and deploy to vrep

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

class VrepDistanceSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_names):
        # variables
        self.client = client
        self.obj_handles = obj_handles
        self.sensor_names = [sensor_names]
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

class VrepTotalSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_mounts_names):
        self.sensor_mounts = [0 for i in range(4)]

        for i in range(4):
            self.sensor_mounts[i] = VrepDistanceSensorBridge(client, obj_handles, sensor_mounts_names[i])

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
        "Proximity_sensor_1",
        "Proximity_sensor_2",
        "Proximity_sensor_3",
        "Proximity_sensor_4"
        ]

    OBJ_NAMES = INDY_OBJ_NAMES + SENSOR_OBJ_NAMES

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
        self.robot = VrepRobotStateBridge(self.client, self.obj_handles, self.INDY_OBJ_NAMES)
        self.total_sensor = VrepTotalSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))


        ### Ros - Moveit
        rate = rospy.Rate(10)
        self.objnames = ['joint0','joint1','joint2','joint3','joint4','joint5']
        self.goal_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cur_pos =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.moveit_cb)#this obtain each joint pos
        self.jointPositions = []
        

        ## default variables
        self.trajectory = []

        # input data
        self.bundle_pos = [0.0 for i in range(4)]
        self.AVG_distance = [0.0 for i in range(4)]

        self.sensor_1_input_temp = []
        self.sensor_1_input_data = []
        self.sensor_2_input_temp = []
        self.sensor_2_input_data = []
        self.sensor_3_input_temp = []
        self.sensor_3_input_data = []
        self.sensor_4_input_temp = []
        self.sensor_4_input_data = []

        # output data
        self.FOV_distance = [0.0 for i in range(4)]
        self.object_check = [0.0 for i in range(4)]

        self.sensor_1_output_temp = []
        self.sensor_1_output_data = []
        self.sensor_2_output_temp = []
        self.sensor_2_output_data = []
        self.sensor_3_output_temp = []
        self.sensor_3_output_data = []
        self.sensor_4_output_temp = []
        self.sensor_4_output_data = []

        self.flag = True
        self.moveit_flag = False
        self.moving_flag = False

    def get_object_handles(self):
        self.obj_handles = dict()
        print(self.OBJ_NAMES)
        for name in self.OBJ_NAMES:
            self.obj_handles[name] = self.client.simxGetObjectHandle(
                name,
                self.client.simxServiceCall()
                )[1]


 ### Call back
  ## Moveit
    def moveit_cb(self, data):
        if self.moving_flag == False:
            jointPoints = data.trajectory[0].joint_trajectory.points #joint pos msg
            for msg in jointPoints:
                self.jointPositions.append(msg.positions) #joint pos list
            
            step_num = int(2800/len(self.jointPositions))

            self.set_trajectory(self.jointPositions, step=step_num)

            self.jointPositions = []
            self.moving_flag = True
            self.moveit_flag = True

  ## Vrep
    def simulation_step_start_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        ### Write the working code ###
        if len(self.trajectory) == 0:
            self.flag = False
            self.moving_flag = False
        else:
            self.robot.excute_pose(list(self.trajectory[0]))
            del self.trajectory[0]

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        for i in range(4):
            self.bundle_pos[i] = self.total_sensor.sensor_mounts[i].sensor_pos
            self.AVG_distance[i] = self.total_sensor.sensor_mounts[i].FOV_distance

            self.FOV_distance[i] = self.total_sensor.sensor_mounts[i].FOV_distance
            self.object_check[i] = self.total_sensor.sensor_mounts[i].FOV_object_check

        if self.flag == False:
        
            # add data set
            self.sensor_1_input_data = self.sensor_1_input_temp
            self.sensor_2_input_data = self.sensor_2_input_temp
            self.sensor_3_input_data = self.sensor_3_input_temp
            self.sensor_4_input_data = self.sensor_4_input_temp

            self.sensor_1_output_data = self.sensor_1_output_temp
            self.sensor_2_output_data = self.sensor_2_output_temp
            self.sensor_3_output_data = self.sensor_3_output_temp
            self.sensor_4_output_data = self.sensor_4_output_temp

            # self.input_data = self.input_temp
            # self.output_data = self.output_temp

            self.sensor_1_learning_input, self.sensor_1_learning_output = make_data(self.sensor_1_input_data, self.sensor_1_output_data)
            self.sensor_2_learning_input, self.sensor_2_learning_output = make_data(self.sensor_2_input_data, self.sensor_2_output_data)
            self.sensor_3_learning_input, self.sensor_3_learning_output = make_data(self.sensor_3_input_data, self.sensor_3_output_data)
            self.sensor_4_learning_input, self.sensor_4_learning_output = make_data(self.sensor_4_input_data, self.sensor_4_output_data)

            # clear temp data
            self.sensor_1_input_temp = []
            self.sensor_2_input_temp = []
            self.sensor_3_input_temp = []
            self.sensor_4_input_temp = []
            self.sensor_1_output_temp = []
            self.sensor_2_output_temp = []
            self.sensor_3_output_temp = []
            self.sensor_4_output_temp = []

            # self.input_temp = []
            # self.output_temp = []

        else:
            self.sensor_1_input_temp += [self.bundle_pos[0] + self.AVG_distance[0]]
            self.sensor_2_input_temp += [self.bundle_pos[1] + self.AVG_distance[1]]
            self.sensor_3_input_temp += [self.bundle_pos[2] + self.AVG_distance[2]]
            self.sensor_4_input_temp += [self.bundle_pos[3] + self.AVG_distance[3]]

            self.sensor_1_output_temp += [self.FOV_distance[0] + self.object_check[0]]
            self.sensor_2_output_temp += [self.FOV_distance[1] + self.object_check[1]]
            self.sensor_3_output_temp += [self.FOV_distance[2] + self.object_check[2]]
            self.sensor_4_output_temp += [self.FOV_distance[3] + self.object_check[3]]

            # self.input_temp = self.input_temp + [bundle_pos + [AVG_distance]]
            # self.output_temp = self.output_temp + [FOV_distance + object_check]
        
        # change do_next_step state
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

    def set_trajectory(self, path, step=20):

        path_length = len(path)

        for path_num in range(path_length-1):
            start_pose = path[path_num]
            end_pose = path[path_num+1]

            step_pose = [((end_pose[i]-start_pose[i])/step) for i in range(6)]

            for pos_num in range(step+1):
                self.trajectory.append([(start_pose[i]+pos_num*step_pose[i]) for i in range(6)])

        self.flag = True

def make_data(input_data, output_data):

    learning_input = []
    learning_output = []

    for i in range(len(input_data)-20):  # data_step = 21
        temp_input = input_data[i:(i+21)]
        temp_output = output_data[(i+20)]

        # print(temp_output)

        learning_input += [sum(temp_input, [])]
        learning_output += [temp_output]

    return learning_input, learning_output

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def rad2deg(rad):
    deg = rad*180/math.pi
    return deg

def save_as_npy(data, name):
    path = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/sonar/PAP_test_1/' + name + '.npy'
    np.save(path, data)

if __name__ == "__main__":
    rospy.init_node('main')

    sensor_1_input_data = []
    sensor_2_input_data = []
    sensor_3_input_data = []
    sensor_4_input_data = []
    sensor_1_output_data = []
    sensor_2_output_data = []
    sensor_3_output_data = []
    sensor_4_output_data = []

    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    save_count = 1

    try:
        while not rospy.is_shutdown():
            if vrep.moveit_flag == True:
                while vrep.flag:
                    vrep.step_simulation()

                sensor_1_input_data += copy.deepcopy(vrep.sensor_1_learning_input)
                sensor_2_input_data += copy.deepcopy(vrep.sensor_2_learning_input)
                sensor_3_input_data += copy.deepcopy(vrep.sensor_3_learning_input)
                sensor_4_input_data += copy.deepcopy(vrep.sensor_4_learning_input)

                sensor_1_output_data += copy.deepcopy(vrep.sensor_1_learning_output)
                sensor_2_output_data += copy.deepcopy(vrep.sensor_2_learning_output)
                sensor_3_output_data += copy.deepcopy(vrep.sensor_3_learning_output)
                sensor_4_output_data += copy.deepcopy(vrep.sensor_4_learning_output)

                sensor_1_input_np_data = np.array(sensor_1_input_data)
                sensor_2_input_np_data = np.array(sensor_2_input_data)
                sensor_3_input_np_data = np.array(sensor_3_input_data)
                sensor_4_input_np_data = np.array(sensor_4_input_data)
                sensor_1_output_np_data = np.array(sensor_1_output_data)
                sensor_2_output_np_data = np.array(sensor_2_output_data)
                sensor_3_output_np_data = np.array(sensor_3_output_data)
                sensor_4_output_np_data = np.array(sensor_4_output_data)

                save_as_npy(sensor_1_input_np_data, 'sensor_1_input_Fold_%d'%(save_count))
                save_as_npy(sensor_2_input_np_data, 'sensor_2_input_Fold_%d'%(save_count))
                save_as_npy(sensor_3_input_np_data, 'sensor_3_input_Fold_%d'%(save_count))
                save_as_npy(sensor_4_input_np_data, 'sensor_4_input_Fold_%d'%(save_count))
                save_as_npy(sensor_1_output_np_data, 'sensor_1_output_Fold_%d'%(save_count))
                save_as_npy(sensor_2_output_np_data, 'sensor_2_output_Fold_%d'%(save_count))
                save_as_npy(sensor_3_output_np_data, 'sensor_3_output_Fold_%d'%(save_count))
                save_as_npy(sensor_4_output_np_data, 'sensor_4_output_Fold_%d'%(save_count))

                print(sensor_1_output_data)

                sensor_1_input_data = []
                sensor_2_input_data = []
                sensor_3_input_data = []
                sensor_4_input_data = []

                sensor_1_output_data = []
                sensor_2_output_data = []
                sensor_3_output_data = []
                sensor_4_output_data = []

                save_count += 1

                vrep.moveit_flag = False

                if save_count > 5:
                    break
            #--------------------------------------------------------------------

        vrep.stop_simulation()

        print(sensor_1_input_np_data.shape)
        print(sensor_1_output_np_data.shape)

    except rospy.ROSInterruptException:
        pass