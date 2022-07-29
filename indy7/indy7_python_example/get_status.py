# from tracemalloc import start
from indy_utils import indydcp_client as client
from indy_utils.indy_program_maker import JsonProgramComponent

import json
import threading
import time


# Set robot (server) IP 
robot_ip = "115.145.159.87"  # Robot (Indy) IP

# Set robot name
name = "NRMK-Indy7"  # Robot name (Indy7)
# name = "NRMK-IndyRP2"  # Robot name (IndyRP2)

# Create class object
indy = client.IndyDCPClient(robot_ip, name)

indy.connect()

status = indy.get_robot_status()
start_time = time.time()
joint_pos = indy.get_joint_pos()
end_time = time.time()
print(1/(end_time - start_time))
task_pos = indy.get_task_pos()

print(status)
print(joint_pos)
print(task_pos)

i=0
while i<100:
    i+=1
    start_time = time.time()
    joint_pos = indy.get_joint_pos()
    end_time = time.time()
    print(1/(end_time - start_time))
    
indy.disconnect()