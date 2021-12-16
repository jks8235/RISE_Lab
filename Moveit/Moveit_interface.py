#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Jin-Jae Shin (RISE)

import sys
import copy
import rospy
import moveit_commander
# import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String,Header
from moveit_commander.conversions import pose_to_list
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import numpy as np
import time

import tf
import copy


def e2q(self,value):
        """
        Convenience method euler to quaternion
        @param: value A float, euler angle
        """
        return tf.transformations.quaternion_from_euler(value[0],value[1],value[2])

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MovegroupInterface(object):
    """
    MoveGroupPythonIntefaceTutorial for indy7
    @param: group_name  string_list : commonly used robot name
    """
    def __init__(self,group_name):
        super(MovegroupInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_interface', anonymous=True)
        robot = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.robot = robot
        self.move_group = move_group

    def go_to_joint_state(self,joint_angles=[0,0,0,0,0,0]):
        """
        Convenience method for jogging the robot in the joint space using joint angles
        @param: joint_angles    A list of floats, each joint angles (the list length is robot DOF)
        """
        # initial joint_angles = zero pose
        move_group = self.move_group
        joint_goal = self.move_group.get_current_joint_values()
        for num,theta in enumerate(joint_angles):
            joint_goal[num] = theta
        move_group.go(joint_goal,wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        # print move_group.get_current_pose()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,pose=[
                                    [0.000145957066851,-0.186500006027,1.27999998635],
                                    [3.4192836842e-09,0.00011971552457,0.0,1,0]
                                    ]):
        """
        Convenience method for jogging the robot in the joint space using joint angles
        @param: pose    A list of floats, [[x,y,z],[qx,qy,qz,qw]] or [[x,y,z],[r(rot_x),p(rot_y),y(rot_z)]]
        """
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x=pose[0][0]
        pose_goal.position.y=pose[0][1]
        pose_goal.position.z=pose[0][2]
        if len(pose[1]) == 4:
            pose_goal.orientation.x=pose[1][0]
            pose_goal.orientation.y=pose[1][1]
            pose_goal.orientation.z=pose[1][2]
            pose_goal.orientation.w=pose[1][3]
        else:
            pose_goal.orientation.x=pose[1][0]
            pose_goal.orientation.y=pose[1][1]
            pose_goal.orientation.z=pose[1][2]
        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal,current_pose,0.01)

    def plan_cartesian_path(self, waypoints, scale=1):
        """
        Convenience plan cartesian path generator
        @param: waypoints [list]
        @param: scale
        """
        move_group = self.move_group
        # print len(waypoints)
        plan, fraction = move_group.compute_cartesian_path(
                                                        waypoints,  # waypoints to follow
                                                        0.01,       # eef_step
                                                        0.0         # jump_threshold
                                                        )
        return plan, fraction
        
    def execute_plan(self, plan):
        """
        display_trajectory 
        @param: plan = path
        """
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def get_way_point(self,point):
        move_group = self.move_group
        way_point = move_group.get_current_pose().pose
        way_point.position.x = point[0]
        way_point.position.y = point[1]
        way_point.position.z = point[2]
        if len(point)==6:
            qu = e2q(point[3:])
        else:
            qu = [0,0,0,0]
            for num,q in enumerate(point[3:]):
                qu[num] = q
        way_point.orientation.x = qu[0]
        way_point.orientation.y = qu[1]
        way_point.orientation.z = qu[2]
        way_point.orientation.w = qu[3]
        return way_point

    def get_plan(self,way_points):
        """
        we should includ a curve algorithm
        way_point에 지점들을 넣고 get_plan으로 plan을 return으로 받아야 한다.

        """
        move_group = self.move_group

        plan_points = []
        vrep_cur_pose = move_group.get_current_pose().pose

        plan_points.append(vrep_cur_pose)

        for way_point in way_points:
            planning_point = self.get_way_point(way_point)
            plan_points.append(planning_point)
        # print len(plan_points)
        # print plan_points
        plan,fraction = self.plan_cartesian_path(plan_points)
        # print len(plan.joint_trajectory.points)
        return plan

    def pick_and_place(self):
        # move_group = self.move_group
        # current_eef_pos = move_group.get_current_pose().pose

        # way_points = [
        #     [current_eef_pos.position.x,
        #     current_eef_pos.position.y,
        #     current_eef_pos.position.z,
        #     current_eef_pos.orientation.x,
        #     current_eef_pos.orientation.y,
        #     current_eef_pos.orientation.z,
        #     current_eef_pos.orientation.w],
        #     [current_eef_pos.position.x+move,
        #     current_eef_pos.position.y,
        #     current_eef_pos.position.z,
        #     current_eef_pos.orientation.x,
        #     current_eef_pos.orientation.y,
        #     current_eef_pos.orientation.z,
        #     current_eef_pos.orientation.w],
        # ]
        way_points = [
            [0.35, -0.45, 0.6, 0.0, -1.0, 0.0, 0.0], # start
            [0.35, -0.45, 0.455, 0.0, -1.0, 0.0, 0.0], # pick
            [0.35, -0.45, 0.55, 0.0, -1.0, 0.0, 0.0], # up
            [0.0, -0.45, 0.55, 0.0, -1.0, 0.0, 0.0], # move
            [-0.35, -0.45, 0.55, 0.0, -1.0, 0.0, 0.0], # place ready
            [-0.35, -0.45, 0.455, 0.0, -1.0, 0.0, 0.0], # place
            [-0.35, -0.45, 0.55, 0.0, -1.0, 0.0, 0.0], # up
            [0.0, -0.45, 0.55, 0.0, -1.0, 0.0, 0.0], # move back
            [0.35, -0.45, 0.55, 0.0, -1.0, 0.0, 0.0], # arrive
            [0.35, -0.45, 0.6, 0.0, -1.0, 0.0, 0.0] # end

            # [-0.5, 0.2, 0.5, 0.0, -1.0, 0.0, 0.0], # start
            # [-0.5, 0.2, 0.3, 0.0, -1.0, 0.0, 0.0], # down & pick
            # [-0.5, 0.2, 0.5, 0.0, -1.0, 0.0, 0.0], # up
            # [-0.5, -0.1, 0.5, 0.0, -1.0, 0.0, 0.0], # move
            # [-0.5, -0.4, 0.5, 0.0, -1.0, 0.0, 0.0], # place position
            # [-0.5, -0.4, 0.3, 0.0, -1.0, 0.0, 0.0], # down & releas
            # [-0.5, -0.4, 0.5, 0.0, -1.0, 0.0, 0.0], # up
            # [-0.5, -0.1, 0.5, 0.0, -1.0, 0.0, 0.0], # back move
            # [-0.5, 0.2, 0.5, 0.0, -1.0, 0.0, 0.0], # arrive
        ]

        #     [start_point.position.x+0.05,start_point.position.y+0.05,start_point.position.z+0.2,current_eef_pos.orientation.x,current_eef_pos.orientation.y,current_eef_pos.orientation.z,current_eef_pos.orientation.w],
        #     [end_point.position.x+0.05,end_point.position.y+0.05,end_point.position.z+0.2,current_eef_pos.orientation.x,current_eef_pos.orientation.y,current_eef_pos.orientation.z,current_eef_pos.orientation.w],
        # ]
        # print len(way_points)
        plan = self.get_plan(way_points)
        # print len(plan.joint_trajectory.points)
        self.execute_plan(plan)




if __name__ == '__main__':
    MI = MovegroupInterface('indy7')
    flag = 0
    while not rospy.is_shutdown():
        # if flag < 3:
        MI.pick_and_place()
        time.sleep(10)
        