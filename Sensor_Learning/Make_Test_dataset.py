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

import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

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
        self.input_data = []
        self.input_temp = []
        self.output_data = []
        self.flag = True
        self.transforamtion_data = []

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
            self.transforamtion_data = self.FOV_distance_sensor.transformation_matrix
            self.output_data = FOV_distance + self.FOV_distance_sensor.FOV_object_check



            # clear temp data
            self.input_temp = []

        else:
            self.input_temp = self.input_temp + bundle_pos + [AVG_distance]

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

    # def pcl_pub_xyz(self,points):
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = 'map'
    #     pcl = point_cloud2.create_cloud_xyz32(header, points)
    #     pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
    #     pointcloud_publisher.publish(pcl)
    #     time.sleep(0.5)

    # def make_point_cloud(self, sensor_data, transformation_matrix):
    #     sensor_num = 17
    #     matrix_value_num = 12
    #     world_points = []

    #     # print(points.shape)
    #     # print(matrixes.shape)

    #     matrixes = matrixes.reshape(12,17,-1)

    #     # print(points.shape[1])
    #     # print(matrixes.shape)
    #     # print(len(matrixes[:,0,0]))

    #     for i in range(points.shape[1]):
    #         for j in range(sensor_num):
    #             point_list = [0, 0, points[j,i], 1]
    #             matrix_list = [matrixes[:,j,i]]

    #             point = np.array(point_list).reshape(4,1)
    #             matrix = np.array(matrix_list).reshape(3,4)

    #             world_point = np.dot(matrix,point)
    #             world_point_list = [world_point[0,0], world_point[1,0], world_point[2,0]]
                
    #             world_points.append(world_point_list)

    #     self.pcl_pub_xyz(world_points)


def _save_data_as_csv(data, name):
    path = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/data/Point_cloud_test_2/' + name + '.csv'
    data.to_csv(path, sep=',', header=None, index=None)

def make_fold(pd_data, fold_num):

    DataNo = pd_data.shape[0]
    input_FeatNo = 168
    matrix_FeatNo = 204 + input_FeatNo
    output_FeatNo = 34 + matrix_FeatNo
    FoldDataNo = int(DataNo/fold_num)

    total_data = pd_data.iloc[np.random.permutation(pd_data.index)]
    total_data = total_data.T
    
    print(total_data.shape)

    ## Validation Data set ##
    for i in range(fold_num):
        
        temp_input_Valid = total_data.iloc[:input_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        temp_matrix_Valid = total_data.iloc[input_FeatNo:matrix_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        temp_output_Valid = total_data.iloc[matrix_FeatNo:output_FeatNo, FoldDataNo*i : FoldDataNo*(i+1)]
        
        
        launch_1 = 'input_Fold%d = temp_input_Valid'%(i+1)
        launch_2 = 'matrix_Fold%d = temp_matrix_Valid'%(i+1)
        launch_3 = 'output_Fold%d = temp_matrix_Valid'%(i+1)

        exec(launch_1)
        exec(launch_2)
        exec(launch_3)

    print(input_Fold1.shape)
    print(matrix_Fold1.shape)
    print(output_Fold1.shape)

    print('fold data done')

    for i in range(0, fold_num):

        launch_1 = '_save_data_as_csv(input_Fold%d, \'input_Fold_%d\')'%(i+1,i+1)
        launch_2 = '_save_data_as_csv(matrix_Fold%d, \'matrix_Fold_%d\')'%(i+1,i+1)
        launch_3 = '_save_data_as_csv(output_Fold%d, \'output_Fold_%d\')'%(i+1,i+1)
        
        exec(launch_1)
        exec(launch_2)
        exec(launch_3)

        print(i+1)

    print ('Save data done')

def save_data(list_data, name, numbering):
    pd_data = pd.DataFrame(list_data)
    data = pd_data.T
    
    _save_data_as_csv(data, '%s%d'%(name,numbering))

    print(numbering)
    # print ('Save data done')

def deg2rad(deg):
    rad = deg*math.pi/180
    return rad

def make_path_set(resolution):
    theta_0_range = range(-30, 30+1 ,resolution)
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

if __name__ == '__main__':
    input_data = []
    matrix_data = []
    output_data = []

    angle_path = make_path_set(30)

    print(len(angle_path))

    vrep = VrepInterface()
    vrep.start_simulation()

    count = 0
    for start_angle, end_angle in angle_path:

        count += 1
        print(count)

        vrep.set_trajectory(start_angle, end_angle)
        
        while vrep.flag:
            vrep.step_simulation()

        # input_temp = vrep.input_data
        # output_temp = vrep.output_data

        # print(vrep.input_data)
        # print(len(vrep.output_data))

        input_data += vrep.input_data
        matrix_data += vrep.transforamtion_data
        output_data += vrep.output_data

    vrep.stop_simulation()

    print(len(matrix_data))

    input_np_data = np.array(input_data).reshape(168,-1)
    matrix_np_data = np.array(matrix_data).reshape(204,-1)
    output_np_data = np.array(output_data).reshape(34,-1)

    print(input_np_data.shape)
    print(matrix_np_data.shape)
    print(output_np_data.shape)

    Total_data = np.concatenate([input_np_data, matrix_np_data, output_np_data], axis=0)
    pd_Total = pd.DataFrame(Total_data).T

    # print(pd_Total.shape)

    make_fold(pd_Total,1)