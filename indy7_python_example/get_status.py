from indy_utils import indydcp_client as client
from indy_utils.indy_program_maker import JsonProgramComponent

import json
import threading
from time import sleep

# Set robot (server) IP 
robot_ip = "10.201.159.232"  # Robot (Indy) IP

# Set robot name
name = "NRMK-Indy7"  # Robot name (Indy7)
# name = "NRMK-IndyRP2"  # Robot name (IndyRP2)

# Create class object
indy = client.IndyDCPClient(robot_ip, name)

indy.connect()

status = indy.get_robot_status()
joint_pos = indy.get_joint_pos()
task_pos = indy.get_task_pos()

print(status)
print(joint_pos)
print(task_pos)

indy.disconnect()