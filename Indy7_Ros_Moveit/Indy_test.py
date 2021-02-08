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

indy.connect()

status = indy.get_robot_status()
print(status)
print status['ready']

#indy.go_home()

# indy.joint_move_to([-13.324600093621745, -63.47151322798713, -52.05036257909693, 0.0007614286819307433, -64.3780880163254, -13.362583878017588])

print(indy.get_joint_pos())
print(indy.get_task_pos())

indy.disconnect()