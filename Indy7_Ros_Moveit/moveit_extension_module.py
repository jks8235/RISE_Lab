#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Jin-Jae Shin (RISE)

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

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

class MoveGroupPythonIntefaceTutorial(object):
    """
    MoveGroupPythonIntefaceTutorial for indy7
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    def __init__(self,group_name):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_test', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = ''
        self.mesh_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

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
        @param: waypoints
        @param: scale
        """
        move_group = self.move_group
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
        move_group.execute(plan, wait=False)

    def display_trajectory(self,plan):
        """
        display_trajectory 
        @param: plan = path
        """
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_mesh(self, obj_name, obj_pose, obj_dir, timeout=4):
        mesh_name = self.mesh_name
        scene = self.scene

        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = "ground"
        mesh_pose.pose.orientation.w = 1.0

        mesh_name = obj_name
        mesh_pose.pose.position.x = obj_pose[0]
        mesh_pose.pose.position.y = obj_pose[1]
        mesh_pose.pose.position.z = obj_pose[2]

        scene.add_mesh(mesh_name, mesh_pose, obj_dir ,size=(0.001, 0.001, 0.001))

        self.mesh_name=mesh_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "ground"
        box_pose.pose.orientation.w = 1.0

        box_pose.pose.position.x = -0.35 # slightly above the end effector
        box_pose.pose.position.y = -0.45 # slightly above the end effector
        box_pose.pose.position.z = 0.36 # slightly above the end effector
        box_name = "box"
        scene.add_mesh(box_name, box_pose, "box.STL",size=(0.001, 0.001, 0.001))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'indy7'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


if __name__ == '__main__':
    tutorial = MoveGroupPythonIntefaceTutorial()
    # tutorial.go_to_joint_state()
    # tutorial.go_to_pose_goal([[0.4,0.1,0.4],[0,0,0,1]])
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()
    tutorial.execute_plan(cartesian_plan)
    # raw_input()
    # tutorial.add_mesh()

