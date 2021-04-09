#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Jin-Jae Shin (RISE)

import numpy as np
import rospy
import math
import sys
import copy
import time
sys.path.insert(0 , '../../library/vrep_lib')
sys.path.insert(0 , '../../library/PointCloud')
from moveit_msgs.msg import DisplayTrajectory
import geometry_msgs.msg
import tf
import moveit_commander
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

#### custom ####
from moveit_extension_module import MoveGroupPythonIntefaceTutorial
import point_cloud_pkg
from vrepsim import VrepSimulation
################

class IndyWithMultiSensor:
    def __init__(self):
        self.vrep_objnames = [
            'joint0',
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'Proximity_sensor1',
            'Proximity_sensor2',
            'Proximity_sensor3',
            'Proximity_sensor4',
            'tcp',
            'customizableTable_tableTop1',
            'customizableTable_tableTop2',
            'Cuboid'
            ]
        self.vrep = VrepSimulation(self.vrep_objnames)
        self.cur_joint_pos =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.moveit_cb)#this obtain each joint pos
        self.rest_joint_pos = []
        self.inv_job=[]
        self.sensor_name = []
        self.point_cloud_module = []
        self.sensor_type = 0
        self.br = tf.TransformBroadcaster()
        self.sensor_config={
            'Proximity_sensor1' : [  0.0, -0.05,  0.0,          0,    math.pi/2, -math.pi/2],
            'Proximity_sensor2' : [ 0.05,   0.0,  0.0,  math.pi/2,    math.pi/2,  math.pi/2],
            'Proximity_sensor3' : [  0.0,  0.05,  0.0,        0.0,    math.pi/2,  math.pi/2],
            'Proximity_sensor4' : [-0.05,   0.0,  0.0, -math.pi/2,    math.pi/2,  math.pi/2]
        }
        self.sensor_data = [0 for _ in range(0,len(self.sensor_config))]
        self.pcl_transform = geometry_msgs.msg.TransformStamped()
        for sensor in self.vrep_objnames:
            if 'Proximity_sensor' in sensor:
                self.sensor_name.append(sensor)
                self.point_cloud_module.append(point_cloud_pkg.ProximitySensor(0.1,120.0,'{}'.format(sensor)))
                # self.point_cloud_module = [pcl_pkg(1),pcl_pkg(2),...]
        
        ############################################################################
        ## for moveit 
        ############################################################################
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "indy7"
        self.MG = MoveGroupPythonIntefaceTutorial(group_name)

        self.box_name = self.MG.box_name
        self.mesh_name = self.MG.mesh_name
        self.scene = self.MG.scene
        self.move_group = self.MG.move_group

        self.environment_obj = ['customizableTable_tableTop1',
                                'customizableTable_tableTop2',
                                ]
        self.header=Header()

    def e2q(self,value):
        """
        Convenience method euler to quaternion
        @param: value A float, euler angle
        """
        return tf.transformations.quaternion_from_euler(value[0],value[1],value[2])

    def moveit_cb(self, data):
        """
        Set queue of path about Vrep robot
        @param : data A DisplayTrajectoryMessage in ros, trajectory msg of result plan in moveit
        """
        jointPoints = data.trajectory[0].joint_trajectory.points #joint pos msg
        for msg in jointPoints:
            self.rest_joint_pos.append(msg.positions) #joint pos list

    def set_sensor_bundle(self,parent_link,tran,ori):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: parent_link  A Str name of parent link in tf tree
        @param: tran     A list of floats, translation from parent link
        @param: ori      A list of floats, rotation from parent link(deg)
        """
        self.br.sendTransform(tran,self.e2q(ori),rospy.Time.now(),'Proximity_sensor_bundle{}'.format(parent_link[-1]),parent_link)

    def PubSensorData(self,frame):
        """
        After converting sensors data into global positions, publish them as pointclouds 
        @param: frame   A list of floats, rotation from sonsor bundle position 
        """

        self.set_sensor_bundle(frame,[0,0,0],[0,0,0])
        for sensor in self.sensor_name:
            self.br.sendTransform(self.sensor_config[sensor][0:3],#Translation
                                self.e2q(self.sensor_config[sensor][3:6]),  #Orientaion euler
                                #(0, 0, 0, 1),                  #Orientation quaternion
                                rospy.Time.now(),
                                sensor,
                                "Proximity_sensor_bundle{}".format(frame[-1]))
            #get sensor data & make pcl
            dtc_state,dtc_point=self.vrep.get_sona_data(sensor)
            # print dtc_point
            if dtc_state:
                dtc_abs=math.sqrt(dtc_point[0]*dtc_point[0]+dtc_point[1]*dtc_point[1]+dtc_point[2]*dtc_point[2])
                sensor_num = self.sensor_name.index(sensor)
                # if  0.3 > dtc_abs and dtc_abs > 0.001:
                #     print "Notice: some object is at {}m distance from {}.".format(dtc_abs,sensor)
                # print self.sensor_data[sensor_num] , dtc_abs
                if self.sensor_type:
                    dtc_abs = dtc_abs-0.05
                self.sensor_data[sensor_num] = copy.deepcopy(dtc_abs)
        
        pcs = []
        for num in range(0,len(self.point_cloud_module)):
            pcl,key = self.point_cloud_module[num].get_pcl(self.sensor_data[num],self.sensor_type)

            # trans,rot = self.listener.lookupTransform('/{}'.format(self.point_cloud_module[num].obj),'Proximity_sensor_bundle',rospy.Time(0))
            
            sensor_name = self.point_cloud_module[num].obj
            pcl_transform = self.pcl_transform
            pcl_transform.header = self.header
            pcl_transform.header.stamp = rospy.Time.now()
            pcl_transform.header.frame_id = frame
            pcl_transform.child_frame_id = copy.deepcopy('/{}'.format(self.point_cloud_module[num].obj))
            pcl_transform.transform.translation.x=self.sensor_config[sensor_name][0]
            pcl_transform.transform.translation.y=self.sensor_config[sensor_name][1]
            pcl_transform.transform.translation.z=self.sensor_config[sensor_name][2]
            sensor_qu = self.e2q(self.sensor_config[sensor_name][3:])
            pcl_transform.transform.rotation.x= sensor_qu[0]
            pcl_transform.transform.rotation.y= sensor_qu[1]
            pcl_transform.transform.rotation.z= sensor_qu[2]
            pcl_transform.transform.rotation.w= sensor_qu[3]
            '''
            do_transform_cloud
            @First param: point cloud data
            @Second param: The Transform message to convert.
            @Second param type: geometry_msgs.msg.TransformStamped
            '''
            cloud_out = do_transform_cloud(pcl, pcl_transform)
            pcs.append(cloud_out)
            
        assemble_pcl = copy.copy(pcs[0])
        for pc in pcs[1:]:
            assemble_pcl.width += pc.width
            assemble_pcl.data += pc.data
          
        self.sensor_data = [0 for _ in range(0,len(self.sensor_config))]
        pointcloud_publisher = rospy.Publisher("proximity_sensor", PointCloud2, queue_size=100)
        pointcloud_publisher.publish(assemble_pcl)

    def get_current_joint_angles(self):
        """
        Current joint angles update in vrep robot
        """
        for n in range(0,len(self.cur_joint_pos)):
            self.cur_joint_pos[n]=self.vrep.get_joint_position('joint%d' % n)
        return self.cur_joint_pos

    def set_joint_target_angles(self,goal_pos):
        """
        Set joint angles update in vrep robot
        """
        for n, p in enumerate(goal_pos,0):
            self.vrep.set_joint_target_position('joint%d' % n, goal_pos[n])

    def run_sim(self):
        """
        Run simulation ros to vrep
        """
        move_group = self.move_group
        if self.vrep.is_not_ready:
            raise NotImplementedError
        else:
            self.vrep.start_simulation()
        rate = rospy.Rate(10)
        self.relation_match()
        plan = self.do_job()
        ## plan is RobotTrajectory msg
        self.rest_joint_pos = copy.deepcopy(plan.joint_trajectory.points)
        
        while not rospy.is_shutdown():
            self.PubSensorData('link6')
            
            if len(self.rest_joint_pos) is 0:
                print "operation completed"
                self.MG.go_to_joint_state(self.get_current_joint_angles())
                # plan = self.do_job()
                self.rest_joint_pos = copy.copy(self.inv_job)
                del self.inv_job[:]
            else:
                goal_pos = self.rest_joint_pos[0].positions # this is tuple(6) of current goal position
            
            if len(self.rest_joint_pos) == len(plan.joint_trajectory.points):
                self.MG.execute_plan(plan)
                pass
            # VREP에서 움직인거 확인하고 ROS에서 움직이는거
            # else:
            #     self.MG.go_to_joint_state(self.get_current_joint_angles())

            if len(self.rest_joint_pos):
                #remaining joint path points
                print "Notice: {} joint positions left.".format(len(self.rest_joint_pos))
                self.inv_job.insert(0,self.rest_joint_pos.pop(0))
                
            
            self.set_joint_target_angles(goal_pos)
            # self.vrep_robot_state_publisher.publish(current_joint_state)
            # demo.launch file [/move_group/fake_controller_joint_states]
            rate.sleep()

    # def pose_list2pose_msg(self,pose_list):
    #     pose = move_group.get_current_pose().pose
        
    #     pose.position.x = pose_list[0]
    #     pose.position.y = pose_list[1]
    #     pose.position.z = pose_list[2]
    #     pose.orientation.x = pose_list[3]
    #     pose.orientation.y = pose_list[4]
    #     pose.orientation.z = pose_list[5]
    #     pose.orientation.w = pose_list[6]

    #     return pose

    def relation_match(self):
        """
        matching vrep pos and ros pos
        """
        move_group = self.move_group
        move_group.clear_pose_targets()
        print 'moveit set initialize'
        vrep_angle = np.round(self.get_current_joint_angles(),4)
        moveit_angle = np.round(move_group.get_current_joint_values(),4)
     
        self.MG.go_to_joint_state(self.get_current_joint_angles())    
        for num in range(0,len(vrep_angle)):
            if abs(vrep_angle[num] - moveit_angle[num]) > 0.001:
                self.relation_match()
        pass

    def do_job(self):
        """
        Set the waypoints for the warking task
        """
        move_group = self.move_group

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

        waypoints.append(copy.deepcopy(wpose))
        start_obj_position = copy.copy(self.vrep.get_object_position('Cuboid'))
        start_obj_position[2] = start_obj_position[2] + 0.05
        start_obj_orientation = [0,np.deg2rad(180),0]
        start_obj_transformation = start_obj_position + start_obj_orientation        
        
        waypoints.append(copy.deepcopy(self.get_waypose(start_obj_transformation)))

        end_point_position = copy.deepcopy(self.vrep.get_object_position('customizableTable_tableTop2'))
        end_point_position[2] = end_point_position[2] + 0.1
        end_point_orientation = start_obj_orientation
        end_point_transformation = end_point_position + end_point_orientation
        
        pick_point = copy.copy(start_obj_transformation)
        pick_point[2] = pick_point[2] + 0.1

        place_point =copy.copy(end_point_transformation)
        place_point[2] = place_point[2] + 0.1

        simple_via_point = (np.array(pick_point) + np.array(place_point))/2

        waypoints.append(copy.deepcopy(self.get_waypose(pick_point)))
        waypoints.append(copy.deepcopy(self.get_waypose(simple_via_point)))
        waypoints.append(copy.deepcopy(self.get_waypose(place_point)))
        waypoints.append(copy.deepcopy(self.get_waypose(end_point_transformation)))

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

    def get_invironment(self):
        """
        Get invironment imformation data
        """
        environment_obj = self.environment_obj
        # 'customizableTable' = length : 0.4, width : 0.4
        # 'customizableTable_tableTop1' = [0.35,-0.45,0.41]
        # 'customizableTable_tableTop2' = [-0.35,-0.45,0.41]

        table_pose = [self.vrep.get_object_position(environment_obj[0]),self.vrep.get_object_position(environment_obj[1])]
        table_pose[0]=[table_pose[0][0]-0.2,table_pose[0][1]-0.2,0]
        table_pose[1]=[table_pose[1][0]-0.2,table_pose[1][1]-0.2,0]

        print 'call invironment'
        self.MG.add_mesh(environment_obj[0],table_pose[0],'./table.STL')
        self.MG.add_mesh(environment_obj[1],table_pose[1],'./table.STL')

        scene = self.scene
        if not len(scene.get_objects()) == len(environment_obj):
            self.get_invironment()

if __name__ == "__main__":
    # rospy.init_node('main')
    # M = IndyWithMultiSensor()
    # rate = rospy.Rate(1)
    # while 1:
    #     M.test_sensor()
    #     rate.sleep()

    rospy.init_node('main')
    M = IndyWithMultiSensor()
    try:
        M.get_invironment()
        M.run_sim()
    except rospy.ROSInterruptException:
        pass