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

# basic pose
indy.go_zero()
indy.go_home()

# joint control, task control

j_pos = indy.get_joint_pos()
t_pos = indy.get_task_pos()
j_rel_pos = [10, 0, 0, 0, 0, 0]
t_rel_pos = [0.01, 0, 0, 0, 0, 0]
print("j_pos : ", j_pos)
print("t_pos : ", t_pos)

# joint control - absolute control
indy.joint_move_to(j_pos)
# joint control - reletive control
indy.joint_move_by(j_rel_pos)

# task control - absolute control
indy.task_move_to(t_pos)
# task control - reletive control
indy.task_move_by(t_rel_pos)

indy.disconnect()