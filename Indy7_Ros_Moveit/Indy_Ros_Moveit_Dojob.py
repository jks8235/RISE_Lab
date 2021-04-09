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
from std_msgs.msg import Header

import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from moveit_extension_module import MoveGroupPythonIntefaceTutorial

# Set robot (server) IP 
robot_ip = "10.201.159.232"  # Robot (Indy) IP

# Set robot name
name = "NRMK-Indy7"  # Robot name (Indy7)
# name = "NRMK-IndyRP2"  # Robot name (IndyRP2)

# Create class object
indy = client.IndyDCPClient(robot_ip, name)

class SingleIndy:
    def __init__(self):
        #rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.moveit_cb)#this obtain each joint pos
        self.ros_msg_joint_points = []
        self.indy7_joint_points = []
        self.cur_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ### For Moveit ###
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "indy7"
        self.MG = MoveGroupPythonIntefaceTutorial(group_name)

        self.move_group = self.MG.move_group
        self.header=Header()
        
    def moveit_cb(self, data):
        print 'cb'
        jointPoints = data.trajectory[0].joint_trajectory.points #joint pos msg
        for msg in jointPoints:
            self.ros_msg_joint_points.append(msg.positions) #joint pos list

    def get_current_joint_angles(self):
        """
        Current joint angles update in indy7
        """
        self.cur_pos = indy.get_joint_pos()

        return self.cur_pos

    def relation_match(self):
        move_group = self.move_group
        move_group.clear_pose_targets()
        print 'moveit set initialize'
        indy7_angle = np.round(self.get_current_joint_angles(),4)
        moveit_angle = np.round(move_group.get_current_joint_values(),4)
        print 1

        # self.MG.go_to_joint_state(self.get_current_joint_angles())
        # for num in range(0, len(indy7_angle)):
        #     if abs(indy7_angle[num] - moveit_angle[num]) > 0.01:
        #         self.relation_match()

    def run_indy7(self, vel, blend):
        move_group = self.move_group
        rate = rospy.Rate(10)

        ### path point setting ###
        # self.relation_match()
        plan = self.do_job()
        ## plan is RobotTrajectory msg
        self.ros_msg_joint_points = copy.deepcopy(plan.joint_trajectory.points)
        
        while not rospy.is_shutdown():
            
            if len(self.ros_msg_joint_points) == 0:
                print "operation completed"
                #self.MG.go_to_joint_state(self.get_current_joint_angles())
                # self.relation_match()

                plan = self.do_job()
                self.ros_msg_joint_points = copy.deepcopy(plan.joint_trajectory.points)
            
            else:
                indy.connect()

                traj = JsonProgramComponent(policy=1, resume_time=3)

                self.set_joint_unit()

                for i in range(len(self.ros_msg_joint_points)):
                    traj.add_joint_move_to(self.indy7_joint_points[i], vel=vel, blend=blend)

                traj_json = traj.program_done()
                indy.set_and_start_json_program(traj_json)
            
                indy.disconnect()

                del self.indy7_joint_points[:]
                del self.ros_msg_joint_points[:]

    def set_joint_unit(self):
        temp_jointpoints = []

        for point in self.ros_msg_joint_points:
            for i, n in enumerate(point.positions):
                temp_jointpoints.append(np.rad2deg(n))
            self.indy7_joint_points.append(temp_jointpoints)
            temp_jointpoints = []

        print self.indy7_joint_points

    def do_job(self):
        """
        Set the waypoints for the warking task
        """
        move_group = self.move_group

        ## waypoints form = [position, cartasian angle] = [x,y,z,x,y,z,w]
        waypoints = []
        wpose = move_group.get_current_pose().pose

        start_point = move_group.get_current_pose()

        wpose.position.x = start_point.pose.position.x
        wpose.position.y = start_point.pose.position.y
        wpose.position.z = start_point.pose.position.z
        wpose.orientation.x = start_point.pose.orientation.x
        wpose.orientation.y = start_point.pose.orientation.y
        wpose.orientation.z = start_point.pose.orientation.z
        wpose.orientation.w = start_point.pose.orientation.w

        ## zero to set position
        zero_to_home_position = [0.54363, -0.18656, 1.10733]
        zero_to_home_orientation = [np.deg2rad(0.02), np.deg2rad(45), np.deg2rad(0.02)]
        zero_to_home_transformation = zero_to_home_position + zero_to_home_orientation

        ## zero to set position2
        zero_to_home_position2 = [0.73866, -0.18657, 0.76260]
        zero_to_home_orientation2 = [np.deg2rad(0.02), np.deg2rad(68.49), np.deg2rad(0.02)]
        zero_to_home_transformation2 = zero_to_home_position2 + zero_to_home_orientation2

        ## set position
        set_position = [0.657, -0.187, 0.3]
        set_orientation = [0, np.deg2rad(-180), 0]
        set_transformation = set_position + set_orientation

        ## start position
        start_obj_position = [0.595, 0.365, 0.387]
        start_obj_orientation = [0, np.deg2rad(-180), 0]
        start_obj_transformation = start_obj_position + start_obj_orientation

        ## end position
        end_point_position = [0.6, -0.361, 0.366]
        end_point_orientation = [0, np.deg2rad(-180), 0]
        end_point_transformation = end_point_position + end_point_orientation
        
        pick_point = copy.copy(start_obj_transformation)
        pick_point[2] = pick_point[2] - 0.1

        place_point =copy.copy(end_point_transformation)
        place_point[2] = place_point[2] - 0.1

        simple_via_point = (np.array(start_obj_transformation) + np.array(end_point_transformation))/2


        waypoints.append(copy.deepcopy(wpose))
        waypoints.append(copy.deepcopy(self.get_waypose(zero_to_home_transformation)))
        waypoints.append(copy.deepcopy(self.get_waypose(zero_to_home_transformation2)))
        # waypoints.append(copy.deepcopy(self.get_waypose(set_transformation)))
        # waypoints.append(copy.deepcopy(self.get_waypose(start_obj_transformation)))
        # waypoints.append(copy.deepcopy(self.get_waypose(pick_point)))
        # waypoints.append(copy.deepcopy(self.get_waypose(start_obj_transformation)))
        # waypoints.append(copy.deepcopy(self.get_waypose(simple_via_point)))
        # waypoints.append(copy.deepcopy(self.get_waypose(end_point_transformation)))
        # waypoints.append(copy.deepcopy(self.get_waypose(place_point)))
        # waypoints.append(copy.deepcopy(self.get_waypose(end_point_transformation)))

        # print start_obj_transformation
        # print simple_via_point
        # print end_point_transformation

        move_group.clear_pose_targets()
        plan = self.MG.plan_cartesian_path(waypoints)[0]
        del waypoints[:]
        return plan

    def get_waypose(self,point):
        """
        Convenience method point list to pose msg
        @param: value A float, euler angle
        """
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        wpose.position.x = point[0]
        wpose.position.y = point[1]
        wpose.position.z = point[2]
        qu = self.e2q(point[3:])
        wpose.orientation.x = qu[0]
        wpose.orientation.y = qu[1]
        wpose.orientation.z = qu[2]
        wpose.orientation.w = qu[3]
        return wpose

    def e2q(self,value):
        """
        Convenience method euler to quaternion
        @param: value A float, euler angle
        """
        return tf.transformations.quaternion_from_euler(value[0],value[1],value[2])

    def go_home(self):
        indy.connect()
        indy.go_home()
        indy.disconnect()


if __name__ == "__main__":
    rospy.init_node('main')
    SI = SingleIndy()
    print 'singgleIndy'

    SI.run_indy7(5, 20)

    # while not SI.ros_msg_joint_points:
    #     try:
    #         pass
    #     except rospy.ROSInterruptException:
    #         pass

    # if SI.ros_msg_joint_points:
    #     # SI.test()
    #     # SI.set_joint_unit()
    #     SI.run_indy7(5, 20)
    #     # SI.home_pos()


    