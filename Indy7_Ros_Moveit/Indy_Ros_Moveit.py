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
        self.jointPositions = []
        
    def moveit_cb(self, data):
        jointPoints = data.trajectory[0].joint_trajectory.points #joint pos msg
        for msg in jointPoints:
            self.jointPositions.append(msg.positions) #joint pos list

    def run_indy7(self):

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            indy.connect()

            vel = 5
            blend = 20

            traj = JsonProgramComponent(policy=1, resume_time=3)

            for i in range(len(self.jointPositions)):
                traj.add_joint_move_to(self.jointPositions[i], vel=vel, blend=blend)

            traj_json = traj.program_done()
            indy.set_and_start_json_program(traj_json)
        
            indy.disconnect()
       
            
if __name__ == "__main__":
    rospy.init_node('main')
    try:
        SI = SingleIndy()
        SI.run_indy7()
    except rospy.ROSInterruptException:
        pass