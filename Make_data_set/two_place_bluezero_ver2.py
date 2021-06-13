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


class Vrep_Interface(object):
    WALL_OBJ_NAMES = [
        "ConcretBlock1",
        "ConcretBlock2"
        ]

    BUNDLE_OBJ_NAMES = [
        "sensor_bundle"
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
        "Proximity_sensor17",
        "Proximity_sensor18",
        "Proximity_sensor19",
        "Proximity_sensor20",
        "Proximity_sensor21",
        "Proximity_sensor22",
        "Proximity_sensor23",
        "Proximity_sensor24",
        "Proximity_sensor25",
        ]
    
    OBJ_NAMES = WALL_OBJ_NAMES + BUNDLE_OBJ_NAMES + SENSOR_OBJ_NAMES

    def __init__(self):
        # vrep client variables
        self.do_next_step = True

        # connect to vrep server
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_proxy', 'b0RemoteApi_proxy')

        # run some command before the synchronous mode
        self.get_object_handles()

        # set simualtion synchronous mode
        self.client.simxSynchronous(True)

        # vrep 

        # # vrep robot bridge
        # self.irb120 = VrepRobotStateBridge(self.client, self.obj_handles, self.IRB_OBJ_NAMES)
        # self.indy7 = VrepRobotStateBridge(self.client, self.obj_handles, self.INDY_OBJ_NAMES)

        # vrep vision sensor bridge
        self.rgb_camera = VrepVisionSensorBridge(self.client, self.obj_handles, "kinect_rgb", mode="rgb")
        self.depth_camera = VrepVisionSensorBridge(self.client, self.obj_handles, "kinect_depth", mode="depth")

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

        self.irb120.excute_trajectory()
        self.indy7.excute_trajectory()

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        # after step done publish ros msgs
        self.irb120_joint_states_pub.publish(self.irb120.joint_states_msg)
        self.irb120_control_state_pub.publish(self.irb120.control_states_msg)
        self.indy7_joint_states_pub.publish(self.indy7.joint_states_msg)
        self.indy7_control_state_pub.publish(self.indy7.control_states_msg)
        self.rgb_image_pub.publish(self.rgb_camera.image_msg)
        self.depth_image_pub.publish(self.depth_camera.image_msg)

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



class DistanceSenseorBridge(object):
    def __init__(self, client, object_handles, object_name, mode, res=[640, 480]):
        # variables
        self.client = client
        self.object_handles = object_handles
        self.object_name = object_name
        self.option = option
        self.width_config = res[0]
        self.height_config = res[1]

        # ROS variables
        self.bridge = CvBridge()

        # make vrep subscriber
        if self.option == "rgb":
            self.client.simxGetVisionSensorImage(self.object_handles[object_name], False, self.client.simxCreateSubscriber(self._image_cb, dropMessages=True))
        elif self.option == "depth":
            self.client.simxGetVisionSensorDepthBuffer(self.object_handles[object_name], True, True, self.client.simxCreateSubscriber(self._image_cb, dropMessages=True))
        else:
            raise AttributeError("option should be `rgb` or `depth`, but get {}".format(self.option))


class VrepVisionSensorBridge(object):
    def __init__(self, client, object_handles, object_name, option, res=[640, 480]):
        # variables
        self.client = client
        self.object_handles = object_handles
        self.object_name = object_name
        self.option = option
        self.width_config = res[0]
        self.height_config = res[1]

        # ROS variables
        self.bridge = CvBridge()

        # make vrep subscriber
        if self.option == "rgb":
            self.client.simxGetVisionSensorImage(self.object_handles[object_name], False, self.client.simxCreateSubscriber(self._image_cb, dropMessages=True))
        elif self.option == "depth":
            self.client.simxGetVisionSensorDepthBuffer(self.object_handles[object_name], True, True, self.client.simxCreateSubscriber(self._image_cb, dropMessages=True))
        else:
            raise AttributeError("option should be `rgb` or `depth`, but get {}".format(self.option))

    def _image_cb(self, msg):
        try:
            self.width = msg[1][0]
            self.height = msg[1][1]
            self.image = msg[2]
        except IndexError:
            self.width = self.width_config
            self.height = self.height_config
            self.image = msg[2]

    @property
    def image_msg(self):
        if self.option == "rgb":
            image = np.frombuffer(self.image, dtype=np.uint8)
            image = np.reshape(image, (self.height, self.width, 3))
            image = np.flip(image, axis=0)
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        if self.option == "depth":
            image = np.frombuffer(self.image, dtype=np.float32)
            image = np.reshape(image, (self.height, self.width))
            image = np.flip(image, axis=0)
            msg = self.bridge.cv2_to_imgmsg(image, "32FC1")
        return msg













with b0RemoteApi.RemoteApiClient('b0RemoteApi_proxy', 'b0RemoteApi_proxy') as client:    
    client.doNextStep = True
    client.runInSynchronousMode = True

    def simulationStepStarted(msg):
        simTime = msg[1][b'simulationTime']
        # print('Simulation step started. Simulation time: ', simTime)

    def simulationStepDone(msg):
        simTime = msg[1][b'simulationTime']
        # print('Simulation step done. Simulation time: ', simTime)
        client.doNextStep = True

    def distanceCallback(msg):
        # print('Received image.', msg[1])
        # print(msg)
        # print(len(msg))
        if msg[1] == 1:
            print(msg[2])
            print(msg[3])
        # print(type(msg[2]))
        # img = np.frombuffer(msg[2], dtype='uint8')
        # print(len(img))

    def stepSimulation():
        if client.runInSynchronousMode:
            while not client.doNextStep:
                client.simxSpinOnce()
            client.doNextStep = False
            client.simxSynchronousTrigger()
        else:
            client.simxSpinOnce()

    # client.simxAddStatusbarMessage('Hello world!', client.simxDefaultPublisher())
    # print("hello wolrd")
    distanceSensorHandle = client.simxGetObjectHandle('Proximity_sensor1', client.simxServiceCall())
    print("get distance sensor handle", distanceSensorHandle)

    if client.runInSynchronousMode:
        client.simxSynchronous(True)

    dedicatedSub=client.simxCreateSubscriber(distanceCallback,1,True)
    client.simxReadProximitySensor(distanceSensorHandle[1], dedicatedSub)
    client.simxReadProximitySensor(distanceSensorHandle[1], client.simxDefaultSubscriber(distanceCallback))
    print("get distance sensor step")

    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    print("check simulation setp started with CB")

    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    print("check simulation setp done with CB")

    client.simxStartSimulation(client.simxDefaultPublisher())
    print("start simalation")

    startTime = time.time()
    while time.time() < startTime + 3:
        stepSimulation()

        # print("curt_time", time.time())
        # print("time_out", startTime + 10)

    client.simxStopSimulation(client.simxDefaultPublisher())

print("done")
