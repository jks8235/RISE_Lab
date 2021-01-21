#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Hong-ryul Jung (RISE)

import time
import rospy
from lib import vrep

# brief
# This Module has vrep commands
# vrep remote api commands (http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)

class VrepSimulation():
    __TIME_OUT = 4.0
    client_id = -1
    handles = {}
    names = []

    def __init__(self, object_names):
        self.names += object_names
        
        # rospy.loginfo('Wait until /use_sim_time==True')
        # while not rospy.is_shutdown():
        #     if rospy.get_param('/use_sim_time'):
        #         break
        # rospy.loginfo('Now /use_sim_time==True')
        
        # Connect to V-REP
        # close all opened connections
        vrep.simxFinish(-1)
        ip, port = ('127.0.0.1', 19997)
        self.is_not_ready = True
       
        # commThreadCycleInMs is 32ms
        self.client_id = vrep.simxStart(ip, port, True, True, 5000, 32)
        if self.client_id == -1:
            rospy.logerr('vrepsim: Connection was failed.')
            self.pause_simulation()
        elif not self.init_handles():
            rospy.logerr('vrepsim: init handles failed.')
            self.pause_simulation()
        else:
            self.is_not_ready = False
    
    def get_joint_position(self, objname):
        err = vrep.simx_return_remote_error_flag
        pose = 0
        while err != vrep.simx_return_ok:
            err, pose = vrep.simxGetJointPosition(
                self.client_id,
                self.handles[objname],
                vrep.simx_opmode_oneshot)
        return pose

    def get_joint_velocity(self, objname):
        
        err = vrep.simx_return_remote_error_flag
        lin_vel=0.0
        ang_vel=0.0
        while err != vrep.simx_return_ok:
            err,lin_vel,ang_vel=vrep.simxGetObjectVelocity(
                self.client_id,
                self.hendles[objname],
                vrep.simx_opmode_oneshot
            )
        return ang_vel

    def get_object_position(self, objname):
        err = vrep.simx_return_remote_error_flag
        pose = 0
        while err != vrep.simx_return_ok:
            err, pose = vrep.simxGetObjectPosition(
                self.client_id,
                self.handles[objname],
                -1,
                vrep.simx_opmode_oneshot)
        return pose

    def get_object_quaternion(self, objname):
        err = vrep.simx_return_remote_error_flag
        quaternion = 0
        while err != vrep.simx_return_ok:
            err, quaternion = vrep.simxGetObjectQuaternion(
                self.client_id,
                self.handles[objname],
                -1,
                vrep.simx_opmode_oneshot)
        return quaternion
    
    ###############
    def set_joint_position(self, objname, radian):
        vrep.simxSetJointPosition(
            self.client_id,
            self.handles[objname],
            radian,
            vrep.simx_opmode_oneshot)

    def set_joint_target_position(self, objname, radian):
        vrep.simxSetJointTargetPosition(
            self.client_id,
            self.handles[objname],
            radian,
            vrep.simx_opmode_oneshot)     

    def get_sona_data(self, objname):
        err = vrep.simx_return_remote_error_flag
        dtc_state=0
        dtc_point=[0.0,0.0,0.0]
        dtc_obj_hendle=0
        dtc_ser_vec=[0.0,0.0,0.0]
        while err != vrep.simx_return_ok:
            err, dtc_state, dtc_point, dtc_obj_hendle, dtc_ser_vec = vrep.simxReadProximitySensor(
                self.client_id,
                self.handles[objname],
                vrep.simx_opmode_oneshot)
        return dtc_point#,force
    ##############
    


    def set_joint_force(self, objname, newton):
        vrep.simxSetJointForce(
             self.client_id,
             self.handles[objname],
             newton, 
             vrep.simx_opmode_oneshot)
         
    def set_joint_target_velocity(self, objname, rad_per_sec):
         vrep.simxSetJointTargetVelocity(
             self.client_id,
             self.handles[objname],
             rad_per_sec,
             vrep.simx_opmode_oneshot)

    def set_communication_pause(self):
        vrep.simxPauseCommunication(self.client_id, True)
    
    def set_communication_resume(self):
        vrep.simxPauseCommunication(self.client_id, False)

    ###############

    def init_handles(self):
        last_name = ''
        try:
            for n in self.names:
                last_name = n
                self.handles[n] = self.get_handle(n)
        except RuntimeError:
            rospy.logerr('There is no name: '+last_name)
            return False
        return True

    def get_handle(self, name):
        err = vrep.simx_return_timeout_flag
        t = time.time()
        handle = 0
        while err != vrep.simx_return_ok:
            err, handle = vrep.simxGetObjectHandle(
                self.client_id, name, vrep.simx_opmode_oneshot_wait)
            if (time.time() - t) > self.__TIME_OUT:
                raise RuntimeError
        return handle

    ###############

    def start_simulation(self):
        try:
            self.send_until_ok(vrep.simxStartSimulation)
            #rospy.loginfo("vrep started")
        except RuntimeError as e:
            rospy.logerr("vrep failed to start: %s" % e)

    def pause_simulation(self):
        try:
            self.send_until_ok(vrep.simxPauseSimulation)
            print("vrep paused")
        except RuntimeError as e:
            rospy.logerr("vrep failed to pause: %s" % e)

    def stop_simulation(self):
        try:
            self.send_until_ok(vrep.simxStopSimulation)
            print("vrep stopped")
        except RuntimeError as e:
            print("vrep failed to stop: %s" % e)

    def send_until_ok(self, func):
        err = vrep.simx_return_timeout_flag
        t = time.time()
        while (err != vrep.simx_return_ok) and (not rospy.is_shutdown()):
            err = func(self.client_id, vrep.simx_opmode_oneshot)
            if (time.time() - t) > self.__TIME_OUT:
                raise RuntimeError

