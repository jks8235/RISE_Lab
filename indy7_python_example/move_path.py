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

# Simple example
indy.connect()

vel = 5
j_blend = 20
t_blend = 0.2

prog = JsonProgramComponent(policy=0, resume_time=2)

# setting waypoints
j_wp1 = [35.45288085937734, -47.414540692797544, -55.27484988378463, -0.0325782700340368, -77.29947836092711, 35.40246340308092]
t_wp1 = [0.6563396438579246, 0.238244127385817, 0.2997290484637074, 0.019571572691017697, 179.9724920327221, 0.05755109533355176]

j_wp2 = [35.453107849627855, -55.09543678977636, -64.7333520306086, -0.03464500502784882, -60.12980546101881, 35.410893506345154]
t_wp2 = [0.6563170228215546, 0.2384558980769108, 0.155472048578118, -0.001384742620016496, 179.94928598028307, 0.06657559817585178]

j_wp3 = [-13.326098229275155, -55.226909542875546, -39.38176430946798, 0.001305306311881274, -85.27680349822653, -13.363236531173529]
t_wp3 = [0.656616407444014, -0.34718812991931075, 0.30052026740751203, 0.02514655643285834, 179.8885157762869, 0.03715637064031851]

j_wp4 = [-13.324600093621745, -63.47151322798713, -52.05036257909693, 0.0007614286819307433, -64.3780880163254, -13.362583878017588]
t_wp4 = [0.6564373136811749, -0.3472904856316228, 0.12218557901739534, 0.02195777089571179, 179.90590167873245, 0.030532790099343992]

# make path
prog.add_move_home()  
prog.add_joint_move_to(j_wp1, vel=vel) 

# Turns on digital output of port indices from 0 to 7 (0: OFF, 1: ON)
# joint control -> task control
for idx in range(0, 8):
    prog.add_digital_out(idx=idx, val=1)

# Wait for set time
prog.add_wait(1)

# Tool command of tool ID and its command
# Tool should be first set up in Conty Application (Setting - Tool)
# In Conty,add tool and set application (e.g. Pick & Place)
# Edit Hold and Release Output and update the tool (refer the Indy manual)
prog.add_endtool_do(type=0, value=0)
prog.add_wait(1)
prog.add_endtool_do(type=0, value=1)

prog.add_task_move_to(t_wp2, vel=vel, blend=t_blend)
prog.add_task_move_to(t_wp3, vel=vel, blend=t_blend)
prog.add_task_move_to(t_wp4, vel=vel, blend=t_blend)

# Turns off digital output of port indices from 0 to 7 (0: OFF, 1: ON)
# task control -> joint control
for idx in range(0, 8):
    prog.add_digital_out(idx=idx, val=0)

prog.add_stop()  # Stop program

indy.set_and_start_json_program(prog.program_done())

indy.disconnect()