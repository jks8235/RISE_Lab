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
import pandas as pd

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
        self.sensor_names = sensor_names
        self.default_distance = 0.15

        self.FOV_distance = [0.0 for _ in range(4)]
        self.FOV_object_check = [0 for _ in range(4)]
        self.sensor_pose = [0.0 for _ in range(4)]

        # create vrep B0 subscriber function
        # Read proximitisensor
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_1, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_2, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_3, dropMessages=True))
        self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._proxi_sensor_cb_4, dropMessages=True))

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], -1, self.client.simxCreateSubscriber(self._pos_cb_1, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[1]], -1, self.client.simxCreateSubscriber(self._pos_cb_2, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[2]], -1, self.client.simxCreateSubscriber(self._pos_cb_3, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[3]], -1, self.client.simxCreateSubscriber(self._pos_cb_4, dropMessages=True))

 ## Call Back
  # Pos Call Back
    def _pos_cb_1(self, msg):
        self.sensor_pose[0] = msg[1]

    def _pos_cb_2(self, msg):
        self.sensor_pose[1] = msg[1]

    def _pos_cb_3(self, msg):
        self.sensor_pose[2] = msg[1]

    def _pos_cb_4(self, msg):
        self.sensor_pose[3] = msg[1]

  # Proxi Sensor Call Back
    def _proxi_sensor_cb_1(self, msg):
        if msg[1] == 1:
            self.FOV_distance[0] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[0] = self.default_distance
            self.FOV_object_check[0] = 0

    def _proxi_sensor_cb_2(self, msg):
        if msg[1] == 1:
            self.FOV_distance[1] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[1] = self.default_distance
            self.FOV_object_check[0] = 0

    def _proxi_sensor_cb_3(self, msg):
        if msg[1] == 1:
            self.FOV_distance[2] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[2] = self.default_distance
            self.FOV_object_check[0] = 0

    def _proxi_sensor_cb_4(self, msg):
        if msg[1] == 1:
            self.FOV_distance[3] = msg[2]
            self.FOV_object_check[0] = 1
        else:
            self.FOV_distance[3] = self.default_distance
            self.FOV_object_check[0] = 0

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
        self.total_sensor = VrepDistanceSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))


        ### Ros - Moveit
        rate = rospy.Rate(10)
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.moveit_cb)#this obtain each joint pos
        self.jointPositions = []
        

        ## default variables
        self.trajectory = []

        # sensor data
        self.sensor_distance = []
        self.sensor_pose = []
        self.sensor_check = []

        # joint position data
        self.joint_pose = []

        self.flag = True
        self.moveit_flag = False
        self.moving_flag = False
        self.stop_flag = False

    def get_object_handles(self):
        self.obj_handles = dict()
        # print(self.OBJ_NAMES)
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

            self.set_trajectory(self.jointPositions, step=10)

            print(self.jointPositions)

            save_as_csv(self.jointPositions, 'target_pose_data')

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

        if self.flag == False:
            pass

        else:
            self.sensor_distance += [copy.deepcopy(self.total_sensor.FOV_distance)]
            self.sensor_check += [copy.deepcopy(self.total_sensor.FOV_object_check)]
            self.sensor_pose += copy.deepcopy([self.total_sensor.sensor_pose[0] + self.total_sensor.sensor_pose[1] + self.total_sensor.sensor_pose[2] + self.total_sensor.sensor_pose[3]])
            self.joint_pose += [copy.deepcopy(self.robot.joint_position)]

            for i in range(4):
                if self.total_sensor.FOV_distance[i] < 0.01:
                    self.flag = False
        
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

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def rad2deg(rad):
    deg = rad*180/math.pi
    return deg

def save_as_npy(list_data, name):
    np_array_data = np.array(list_data)
    print(np_array_data.shape)
    path = '/media/jee/FC12-B7D8/data_folder/Best_sensor_position/No_obstacle_' + name + '.npy'
    np.save(path, np_array_data)

def save_as_csv(list_data, name):
    df_data = pd.DataFrame(np.array(list_data))
    path = '/media/jee/FC12-B7D8/data_folder/Best_sensor_position/No_Obstacle_' + name + '.csv'
    df_data.to_csv(path, index=False)

if __name__ == "__main__":
    rospy.init_node('main')

    sensor_distance_data = []
    sensor_check_data = []
    sensor_pose_data = []
    joint_pose_data = []

    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    save_count = 1

    try:
        while not rospy.is_shutdown():
            if vrep.moveit_flag == True:
                while vrep.flag:
                    vrep.step_simulation()

                sensor_distance_data = copy.deepcopy(vrep.sensor_distance)
                sensor_check_data = copy.deepcopy(vrep.sensor_check)
                sensor_pose_data = copy.deepcopy(vrep.sensor_pose)
                joint_pose_data = copy.deepcopy(vrep.joint_pose)

                # print(sensor_pose_data)

                # save_as_npy(sensor_distance_data, 'distance_data')
                # save_as_npy(sensor_check_data, 'check_data')
                # save_as_npy(sensor_pose_data, 'sensor_pose')
                # save_as_npy(joint_pose_data, 'joint_pose_data')

                save_as_csv(sensor_distance_data, 'distance_data')
                save_as_csv(sensor_check_data, 'check_data')
                save_as_csv(sensor_pose_data, 'sensor_pose')
                save_as_csv(joint_pose_data, 'joint_pose_data')

                save_count += 1

                vrep.moveit_flag = False

                if save_count > 0:
                    break
            #--------------------------------------------------------------------

        vrep.stop_simulation()

        print(len(sensor_distance_data))
        print(len(sensor_check_data))
        print(len(sensor_pose_data))
        print(len(joint_pose_data))

    except rospy.ROSInterruptException:
        pass