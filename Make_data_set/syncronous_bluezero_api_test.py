#!/usr/bin/python
# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/synchronousImageTransmissionViaRemoteApi.ttt
#
# Do not launch simulation, and make sure that the B0 resolver
# is running. Then run this script
#
# The client side (i.e. this script) depends on:
#
# b0RemoteApi (Python script), which depends several libraries present
# in the CoppeliaSim folder

import time
import pdb
import numpy as np
import sys

sys.path.insert(0 , '/home/jee/work_space/catkin_wk/src/RISE_Lab/Library/')
from CoppeliaSim_bluezero import b0RemoteApi

with b0RemoteApi.RemoteApiClient('b0RemoteApi_V-REP', 'b0RemoteApi') as client:    
    client.doNextStep = True
    client.runInSynchronousMode = True

    def simulationStepStarted(msg):
        simTime = msg[1][b'simulationTime']
        # print('Simulation step started. Simulation time: ', simTime)

    def simulationStepDone(msg):
        simTime = msg[1][b'simulationTime']
        # print('Simulation step done. Simulation time: ', simTime)
        client.doNextStep = True

    def imageCallback(msg):
        # print('Received image.', msg[1])
        # print(msg[2][0:3])
        # print(msg)
        # print(type(msg[2]))
        # img = np.frombuffer(msg[2], dtype='uint8')
        # print(len(img))
        client.simxSetVisionSensorImage(passiveVisionSensorHandle[1], False, msg[2], client.simxDefaultPublisher())

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
    visionSensorHandle = client.simxGetObjectHandle('VisionSensor', client.simxServiceCall())
    print("get vistion sensor handle", visionSensorHandle)
    passiveVisionSensorHandle = client.simxGetObjectHandle('PassiveVisionSensor', client.simxServiceCall())
    print("get passive sensor handle", passiveVisionSensorHandle)

    if client.runInSynchronousMode:
        client.simxSynchronous(True)

    dedicatedSub=client.simxCreateSubscriber(imageCallback,1,True)
    client.simxGetVisionSensorImage(visionSensorHandle[1],False,dedicatedSub)
    client.simxGetVisionSensorImage(visionSensorHandle[1], False, client.simxDefaultSubscriber(imageCallback))
    print("get vistion sensor image step")

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
