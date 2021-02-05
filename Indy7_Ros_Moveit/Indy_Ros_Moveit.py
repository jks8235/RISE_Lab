#!/usr/bin/env python
# -*- coding: utf-8 -*-

from indy_utils import indydcp_client as client
from indy_utils.indy_program_maker import JsonProgramComponent

import json
import threading
from time import sleep

import numpy as np
import rospy
import time
import math

from moveit_msgs.msg import DisplayTrajectory

# Set robot (server) IP 
robot_ip = "10.201.159.232"  # Robot (Indy) IP

# Set robot name
name = "NRMK-Indy7"  # Robot name (Indy7)
# name = "NRMK-IndyRP2"  # Robot name (IndyRP2)

# Create class object
indy = client.IndyDCPClient(robot_ip, name)

class SingleIndy:
    def __init__(self):
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.moveit_cb)#this obtain each joint pos
        self.ros_msg_joint_points = []
        self.indy7_joint_points = []
        
    def moveit_cb(self, data):
        jointPoints = data.trajectory[0].joint_trajectory.points #joint pos msg
        for msg in jointPoints:
            self.ros_msg_joint_points.append(msg.positions) #joint pos list

    def run_indy7(self, vel, blend):

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            if len(self.ros_msg_joint_points) == 0:
                pass
                #print 'moveit set initialize'
            
            else:
                indy.connect()

                self.set_joint_unit()

                traj = JsonProgramComponent(policy=1, resume_time=3)

                for i in range(len(self.ros_msg_joint_points)):
                    traj.add_joint_move_to(self.indy7_joint_points[i], vel=vel, blend=blend)

                traj_json = traj.program_done()
                indy.set_and_start_json_program(traj_json)
            
                indy.disconnect()

                print self.indy7_joint_points

                del self.indy7_joint_points[:]
                del self.ros_msg_joint_points[:]

    def set_joint_unit(self):
        temp_jointpoints = []
        for point in self.ros_msg_joint_points:
            for i, n in enumerate(point):
                temp_jointpoints.append(np.rad2deg(n))
            self.indy7_joint_points.append(temp_jointpoints)
            temp_jointpoints = []


if __name__ == "__main__":
    rospy.init_node('main')
    SI = SingleIndy()
    while not SI.ros_msg_joint_points:
        try:
            pass
        except rospy.ROSInterruptException:
            pass

    if SI.ros_msg_joint_points:
        # SI.test()
        # SI.set_joint_unit()
        SI.run_indy7(5, 20)
        # SI.home_pos()


    