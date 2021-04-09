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
import tf
import sys
import copy

### indy7 set up ###
# Set robot (server) IP 
robot_ip = "10.201.159.232"  # Robot (Indy) IP

# Set robot name
name = "NRMK-Indy7"  # Robot name (Indy7)
# name = "NRMK-IndyRP2"  # Robot name (IndyRP2)

# Create indy7 class object
indy = client.IndyDCPClient(robot_ip, name)

### robot move class
class SingleIndy:
    def __init__(self):
        # initialize moving point
        self.indy7_joint_pos = []
        self.indy7_task_pos = []

        # motion velocity, blend setting
        self.vel = 5
        self.j_blend = 20
        self.t_blend = 0.2

    # running code
    def run_indy7(self):

        self.do_job()

        indy.connect()

        path = self.make_path('mix')
        indy.set_and_start_json_program(path)


        iterate = 0
        # repeat pick and place motion
        """
        로봇 모션이 끝나기 전에 다시 코드가 돌면 inhashble error 가 발생하므로 동작이 다 끝나면 (not busy state)
        코드가 시작하도록 설정
        """
        while True:
            if not indy.get_robot_status()['busy'] and indy.get_robot_status()['movedone']:
                time.sleep(1)
                indy.set_and_start_json_program(path)

                iterate += 1
                if iterate == 5:
                    break
                # time.sleep(5)

        indy.disconnect()

        # initialize moving point
        del self.indy7_joint_pos[:]
        del self.indy7_task_pos[:]

    ### setting moving point
    def do_job(self):
        # pick ready pos
        pick_ready_joint_pos = [35.45288085937734, -47.414540692797544, -55.27484988378463, -0.0325782700340368, -77.29947836092711, 35.40246340308092]
        pick_ready_task_pos = [0.6563396438579246, 0.238244127385817, 0.2997290484637074, 0.019571572691017697, 179.9724920327221, 0.05755109533355176]
        
        # pick pos
        pick_joint_pos = [35.453107849627855, -55.09543678977636, -64.7333520306086, -0.03464500502784882, -60.12980546101881, 35.410893506345154]
        pick_task_pos = [0.6563170228215546, 0.2384558980769108, 0.155472048578118, -0.001384742620016496, 179.94928598028307, 0.06657559817585178]

        # release ready pos
        release_ready_joint_pos = [-13.326098229275155, -55.226909542875546, -39.38176430946798, 0.001305306311881274, -85.27680349822653, -13.363236531173529]
        release_ready_task_pos = [0.656616407444014, -0.34718812991931075, 0.30052026740751203, 0.02514655643285834, 179.8885157762869, 0.03715637064031851]

        # release pos
        release_joint_pos = [-13.324600093621745, -63.47151322798713, -52.05036257909693, 0.0007614286819307433, -64.3780880163254, -13.362583878017588]
        release_task_pos = [0.6564373136811749, -0.3472904856316228, 0.12218557901739534, 0.02195777089571179, 179.90590167873245, 0.030532790099343992]

        self.indy7_joint_pos.append(pick_ready_joint_pos)
        self.indy7_joint_pos.append(pick_joint_pos)
        self.indy7_joint_pos.append(pick_ready_joint_pos)

        self.indy7_joint_pos.append(release_ready_joint_pos)
        self.indy7_joint_pos.append(release_joint_pos)
        self.indy7_joint_pos.append(release_ready_joint_pos)

        self.indy7_task_pos.append(pick_ready_task_pos)
        self.indy7_task_pos.append(pick_task_pos)
        self.indy7_task_pos.append(pick_ready_task_pos)

        self.indy7_task_pos.append(release_ready_task_pos)
        self.indy7_task_pos.append(release_task_pos)
        self.indy7_task_pos.append(release_ready_task_pos)

    ### make path
    # para switch : string type; path를 joint state, task state, mix state중 어떤것을 사용할지 선택
    def make_path(self,switch):
        prog = JsonProgramComponent(policy=0, resume_time=2)
        
        if switch == 'joint':
            prog.add_move_home()
            prog.add_joint_move_to(self.indy7_joint_pos[0], vel=self.vel, blend=self.j_blend)
            prog.add_joint_move_to(self.indy7_joint_pos[1], vel=self.vel, blend=self.j_blend)
            prog.add_joint_move_to(self.indy7_joint_pos[2], vel=self.vel, blend=self.j_blend)
            prog.add_joint_move_to(self.indy7_joint_pos[3], vel=self.vel, blend=self.j_blend)
            prog.add_joint_move_to(self.indy7_joint_pos[4], vel=self.vel, blend=self.j_blend)
            prog.add_joint_move_to(self.indy7_joint_pos[5], vel=self.vel, blend=self.j_blend)
            prog.add_move_home()
            prog.add_stop()

        elif switch == 'task':

            prog.add_task_move_to(self.indy7_task_pos[1], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[2], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[0], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[3], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[4], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[5], vel=self.vel, blend=self.t_blend)
            prog.add_stop()

        else:
            prog.add_move_home()
            prog.add_joint_move_to(self.indy7_joint_pos[0], vel=self.vel, blend=self.j_blend)

            self.set_control_config('task', prog)
            prog.add_task_move_to(self.indy7_task_pos[1], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[2], vel=self.vel, blend=self.t_blend)

            self.set_control_config('joint', prog)
            prog.add_joint_move_to(self.indy7_joint_pos[3], vel=self.vel, blend=self.j_blend)

            self.set_control_config('task', prog)
            prog.add_task_move_to(self.indy7_task_pos[4], vel=self.vel, blend=self.t_blend)
            prog.add_task_move_to(self.indy7_task_pos[5], vel=self.vel, blend=self.t_blend)

            self.set_control_config('joint', prog)

            prog.add_move_home()
            prog.add_stop()

        path = prog.program_done()

        return path

    ### reset trajectory option : how to control joint or task control
    """
    indy7은 task와 joint point를 섞어 쓸 경우, 데이터가 바뀌었다는 것을 알려줘야 하므로 사용함 
    """
    def set_control_config(self, control, prog):
    # para control : string type;
    # Turns on digital output of port indices from 0 to 7 (0: joint, 1: task)
    # para prog : dick type;
        if control == 'task':
            for idx in range(0, 8):
                prog.add_digital_out(idx=idx, val=1)
        
        else:
            for idx in range(0, 8):
               prog.add_digital_out(idx=idx, val=0)

if __name__ == "__main__":
    SI = SingleIndy()
    print 'singgleIndy'

    SI.run_indy7()

    