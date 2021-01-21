#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Hong-ryul Jung (RISE)

import numpy as np
import rospy
import time
import math
from vrepsim import VrepSimulation

from moveit_msgs.msg import DisplayTrajectory


# brief
# The program connects moveit with vrep
# Get information about each joint value from movegroup / display_planed_path of moveit and deploy to vrep
class SingleIndy:
    def __init__(self):
        self.objnames = ['joint0','joint1','joint2','joint3','joint4','joint5']
        self.vrep = VrepSimulation(self.objnames)
        self.goal_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cur_pos =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.moveit_cb)#this obtain each joint pos
        self.jointPositions = []
        
    def moveit_cb(self, data):
        jointPoints = data.trajectory[0].joint_trajectory.points #joint pos msg
        for msg in jointPoints:
            self.jointPositions.append(msg.positions) #joint pos list

    def run_sim(self):
        if self.vrep.is_not_ready:
            raise NotImplementedError
        else:
            self.vrep.start_simulation()

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            if len(self.jointPositions) is 0:
                # is not shutdown
                print "operation completed"
                continue
                # is shutdown
                #break
            else:
                goal_pos = self.jointPositions[0] # this is tuple(6) of current goal position
                self.jointPositions.pop(0)
            
            #remaining joint path points
            print "Notice: {} joint positions left.".format(len(self.jointPositions))
            self.jointPositions.pop(0)

            # goal position is in [0,2*pi)
            
            #for n, p in enumerate(goal_pos,0):
            #    goal_pos[n] = p%(math.pi*2.0)

            threshold = 0.2

            # iterate for all joints
            for n, p in enumerate(goal_pos,0):
                # check the error is in threshold boundary
                self.vrep.set_joint_target_position('joint%d' % n, goal_pos[n])
            '''
            for n, p in enumerate(goal_pos,0):
                while abs(goal_pos[n]-self.cur_pos[n]) > threshold:
                    print "WARNING: pos threashold[{}] is out of boundary".format(n)
                    print "    current pos[{}] : {}".format(n,self.cur_pos[n])
                    print "    goal    pos[{}] : {}".format(n,p)
            '''
            

            rate.sleep()

            
            
if __name__ == "__main__":
    rospy.init_node('main')
    try:
        SI = SingleIndy()
        SI.run_sim()
    except rospy.ROSInterruptException:
        pass