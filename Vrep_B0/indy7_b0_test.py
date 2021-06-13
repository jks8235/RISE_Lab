#!/usr/bin/python
#-*- coding: utf-8 -*-
import time
import pdb # good package
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

import sys

sys.path.insert(0 , '/home/jee/work_space/catkin_wk/src/RISE_Lab/Library/')
from CoppeliaSim_bluezero import b0RemoteApi

### msg
from std_msgs.msg import Float32MultiArray


class VrepRobotStateBridge(object):
    def __init__(self, client, obj_handles, joint_names):
        self.client = client
        self.obj_handles = obj_handles
        self.joint_names = joint_names

        self.joint_position = [0.0 for i in range(6)]
        self.trajectory_queue = []

        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[0]], self.client.simxCreateSubscriber(self._joint1_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[1]], self.client.simxCreateSubscriber(self._joint2_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[2]], self.client.simxCreateSubscriber(self._joint3_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[3]], self.client.simxCreateSubscriber(self._joint4_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[4]], self.client.simxCreateSubscriber(self._joint5_cb, dropMessages=True))
        self.client.simxGetJointPosition(self.obj_handles[self.joint_names[5]], self.client.simxCreateSubscriber(self._joint6_cb, dropMessages=True))

        # for num in range(len(self.joint_names)):
        #     self.joint_num = num
        #     print(self.joint_num)
        #     self.client.simxGetJointPosition(self.obj_handles[self.joint_names[num]], self.client.simxCreateSubscriber(self._joint_cb, dropMessages=True))

        # create pub topics
        self.joint_state_pub_topics = dict()
        for name in joint_names:
            self.joint_state_pub_topics[name] = self.client.simxCreatePublisher(True)

    # def _joint_cb(self, msg):
    #     self.joint_position[self.joint_num] = msg[1]

    def _joint1_cb(self, msg):
        self.joint_position[0] = msg[1]

    def _joint2_cb(self, msg):
        self.joint_position[1] = msg[1]

    def _joint3_cb(self, msg):
        self.joint_position[2] = msg[1]

    def _joint4_cb(self, msg):
        self.joint_position[3] = msg[1]

    def _joint5_cb(self, msg):
        self.joint_position[4] = msg[1]

    def _joint6_cb(self, msg):
        self.joint_position[5] = msg[1]

    # @property
    # def joint_states_msg(self):
    #     # make joint_states msg
    #     msg = JointState()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.name = self.joint_names
    #     msg.position = self.joint_position
    #     return msg

    # @property
    # def control_states_msg(self):
    #     # make control msg
    #     msg = FollowJointTrajectoryFeedback()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.joint_names = self.joint_names
    #     msg.actual.positions = self.joint_position
    #     msg.desired.positions = self.joint_position
    #     msg.error.positions = [0, 0, 0, 0, 0, 0]
    #     return msg

    # def ros_joint_path_cb(self, msg):
    #     trajectory_pos = []
    #     trajectory_time = []
    #     if msg.points:
    #         # get trajectory information
    #         trajectory_time = [p.time_from_start.secs + p.time_from_start.nsecs*1e-9 for p in msg.points]
    #         trajectory_pos = [p.positions for p in msg.points]

    #         # make numpy
    #         trajectory_time = np.array(trajectory_time)
    #         trajectory_pos = np.array(trajectory_pos).transpose()

    #         # adjust trajectory time to 0.05 * n
    #         trajectory_time[-1] += 0.05 - (trajectory_time[-1] % 0.05)
    #         trajectory_time_new = np.arange(0, trajectory_time[-1]+0.001, 0.05)

    #         # cubic interpolation
    #         if len(trajectory_time) < 3:
    #             f = interpolate.interp1d(trajectory_time, trajectory_pos, kind='linear')
    #         else:
    #             f = interpolate.interp1d(trajectory_time, trajectory_pos, kind='cubic')
    #         trajectory_pos_new = f(trajectory_time_new).transpose()
    #         trajectory_pos_new = np.flip(trajectory_pos_new, axis=0)

    #         # make excutable trajectory
    #         self.trajectory_queue = trajectory_pos_new.tolist()

    def excute_trajectory(self):
        # if the queue is not empty, set desired pos.
        if self.trajectory_queue:
            desired_joint_positions = self.trajectory_queue.pop()

            bundle = zip(self.joint_names, desired_joint_positions)
            for name, desired_position in bundle:
                self.client.simxSetJointTargetPosition(self.obj_handles[name], desired_position, self.joint_state_pub_topics[name])

class VrepDistanceSensorBridge(object):
    def __init__(self, client, obj_handles, sensor_names):
        # variables
        self.client = client
        self.obj_handles = obj_handles
        self.sensor_names = sensor_names
        self.default_distance = 2.0
        self.distance = [self.default_distance for i in range(len(self.sensor_names))]
        # print(self.obj_handles)

        # create vrep B0 subscriber
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[0]], self.client.simxCreateSubscriber(self._distance_cb_1, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[1]], self.client.simxCreateSubscriber(self._distance_cb_2, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[2]], self.client.simxCreateSubscriber(self._distance_cb_3, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[3]], self.client.simxCreateSubscriber(self._distance_cb_4, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[4]], self.client.simxCreateSubscriber(self._distance_cb_5, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[5]], self.client.simxCreateSubscriber(self._distance_cb_6, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[6]], self.client.simxCreateSubscriber(self._distance_cb_7, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[7]], self.client.simxCreateSubscriber(self._distance_cb_8, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[8]], self.client.simxCreateSubscriber(self._distance_cb_9, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[9]], self.client.simxCreateSubscriber(self._distance_cb_10, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[10]], self.client.simxCreateSubscriber(self._distance_cb_11, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[11]], self.client.simxCreateSubscriber(self._distance_cb_12, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[12]], self.client.simxCreateSubscriber(self._distance_cb_13, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[13]], self.client.simxCreateSubscriber(self._distance_cb_14, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[14]], self.client.simxCreateSubscriber(self._distance_cb_15, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[15]], self.client.simxCreateSubscriber(self._distance_cb_16, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[16]], self.client.simxCreateSubscriber(self._distance_cb_17, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[17]], self.client.simxCreateSubscriber(self._distance_cb_18, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[18]], self.client.simxCreateSubscriber(self._distance_cb_19, dropMessages=True))
        # self.client.simxReadProximitySensor(self.obj_handles[self.sensor_names[19]], self.client.simxCreateSubscriber(self._distance_cb_20, dropMessages=True))

    def get_rel_sensor_transformation(self,parent_frame):
        self.rel=[]
        self.temp_rel = [0 for i in range(len(self.sensor_names))]

        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[0]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_1, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[1]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_2, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[2]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_3, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[3]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_4, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[4]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_5, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[5]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_6, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[6]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_7, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[7]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_8, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[8]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_9, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[9]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_10, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[10]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_11, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[11]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_12, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[12]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_13, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[13]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_14, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[14]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_15, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[15]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_16, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[16]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_17, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[17]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_18, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[18]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_19, dropMessages=True))
        self.client.simxGetObjectPose(self.obj_handles[self.sensor_names[19]], self.obj_handles[parent_frame], self.client.simxCreateSubscriber(self._rel_pos_cb_20, dropMessages=True))

    @property
    def rel_pos_msg(self):
        # make control msg
        msg = Float32MultiArray()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = self.joint_names
        msg.actual.positions = self.joint_position
        msg.desired.positions = self.joint_position
        msg.error.positions = [0, 0, 0, 0, 0, 0]
        return msg

        # simxGetObjectPosition(self,objectHandle,relObjHandle,topic):


        # for obj_name in self.SENSOR_OBJ_NAMES:
        #     rel+=(vrep.simxGetObjectPosition(self.client_id,self.obj_handles[obj_name],self.obj_handles[parent_frame],vrep.simx_opmode_buffer)[1])
        #     rel+=(vrep.simxGetObjectQuaternion(self.client_id,self.obj_handles[obj_name],self.obj_handles[parent_frame],vrep.simx_opmode_buffer)[1])
        #     rel_names.append(parent_frame)
        #     rel_names.append(obj_name)
        #     #rel_names 두개씩 끊어 읽어야 함
        # self.vrep_msg.rel_names = rel_names
        # self.sensor_transformation_msg = Float32MultiArray()
        # self.sensor_transformation_msg.data = rel

    def _rel_pos_cb_1(self, msg):
        self.temp_rel[0] = msg[1]
    
    def _rel_pos_cb_2(self, msg):
        self.temp_rel[1] = msg[1]

    def _rel_pos_cb_3(self, msg):
        self.temp_rel[2] = msg[1]

    def _rel_pos_cb_4(self, msg):
        self.temp_rel[3] = msg[1]

    def _rel_pos_cb_5(self, msg):
        self.temp_rel[4] = msg[1]
    
    def _rel_pos_cb_6(self, msg):
        self.temp_rel[5] = msg[1]
    
    def _rel_pos_cb_7(self, msg):
        self.temp_rel[6] = msg[1]
    
    def _rel_pos_cb_8(self, msg):
        self.temp_rel[7] = msg[1]
    
    def _rel_pos_cb_9(self, msg):
        self.temp_rel[8] = msg[1]
    
    def _rel_pos_cb_10(self, msg):
        self.temp_rel[9] = msg[1]
    
    def _rel_pos_cb_11(self, msg):
        self.temp_rel[10] = msg[1]
    
    def _rel_pos_cb_12(self, msg):
        self.temp_rel[11] = msg[1]
    
    def _rel_pos_cb_13(self, msg):
        self.temp_rel[12] = msg[1]
    
    def _rel_pos_cb_14(self, msg):
        self.temp_rel[13] = msg[1]
    
    def _rel_pos_cb_15(self, msg):
        self.temp_rel[14] = msg[1]
    
    def _rel_pos_cb_16(self, msg):
        self.temp_rel[15] = msg[1]
    
    def _rel_pos_cb_17(self, msg):
        self.temp_rel[16] = msg[1]
    
    def _rel_pos_cb_18(self, msg):
        self.temp_rel[17] = msg[1]
    
    def _rel_pos_cb_19(self, msg):
        self.temp_rel[18] = msg[1]
    
    def _rel_pos_cb_20(self, msg):
        self.temp_rel[19] = msg[1]
    


    def _distance_cb_1(self, msg):
        if msg[1] == 1:
            self.distance[0] = msg[2]
        else:
            self.distance[0] = self.default_distance

    def _distance_cb_2(self, msg):
        if msg[1] == 1:
            self.distance[1] = msg[2]
        else:
            self.distance[1] = self.default_distance

    def _distance_cb_3(self, msg):
        if msg[1] == 1:
            self.distance[2] = msg[2]
        else:
            self.distance[2] = self.default_distance

    def _distance_cb_4(self, msg):
        if msg[1] == 1:
            self.distance[3] = msg[2]
        else:
            self.distance[3] = self.default_distance

    def _distance_cb_5(self, msg):
        if msg[1] == 1:
            self.distance[4] = msg[2]
        else:
            self.distance[4] = self.default_distance

    def _distance_cb_6(self, msg):
        if msg[1] == 1:
            self.distance[5] = msg[2]
        else:
            self.distance[5] = self.default_distance

    def _distance_cb_7(self, msg):
        if msg[1] == 1:
            self.distance[6] = msg[2]
        else:
            self.distance[6] = self.default_distance

    def _distance_cb_8(self, msg):
        if msg[1] == 1:
            self.distance[7] = msg[2]
        else:
            self.distance[7] = self.default_distance

    def _distance_cb_9(self, msg):
        if msg[1] == 1:
            self.distance[8] = msg[2]
        else:
            self.distance[8] = self.default_distance

    def _distance_cb_10(self, msg):
        if msg[1] == 1:
            self.distance[9] = msg[2]
        else:
            self.distance[9] = self.default_distance

    def _distance_cb_11(self, msg):
        if msg[1] == 1:
            self.distance[10] = msg[2]
        else:
            self.distance[10] = self.default_distance

    def _distance_cb_12(self, msg):
        if msg[1] == 1:
            self.distance[11] = msg[2]
        else:
            self.distance[11] = self.default_distance

    def _distance_cb_13(self, msg):
        if msg[1] == 1:
            self.distance[12] = msg[2]
        else:
            self.distance[12] = self.default_distance

    def _distance_cb_14(self, msg):
        if msg[1] == 1:
            self.distance[13] = msg[2]
        else:
            self.distance[13] = self.default_distance

    def _distance_cb_15(self, msg):
        if msg[1] == 1:
            self.distance[14] = msg[2]
        else:
            self.distance[14] = self.default_distance

    def _distance_cb_16(self, msg):
        if msg[1] == 1:
            self.distance[15] = msg[2]
        else:
            self.distance[15] = self.default_distance

    def _distance_cb_17(self, msg):
        if msg[1] == 1:
            self.distance[16] = msg[2]
        else:
            self.distance[16] = self.default_distance

    def _distance_cb_18(self, msg):
        if msg[1] == 1:
            self.distance[17] = msg[2]
        else:
            self.distance[17] = self.default_distance

    def _distance_cb_19(self, msg):
        if msg[1] == 1:
            self.distance[18] = msg[2]
        else:
            self.distance[18] = self.default_distance

    def _distance_cb_20(self, msg):
        if msg[1] == 1:
            self.distance[19] = msg[2]
        else:
            self.distance[19] = self.default_distance

class VrepInterface(object):

    INDY_OBJ_NAMES = [
        "joint0",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        ]

    SENSOR_OBJ_NAMES = [
        "Proximity_sensor1",
        "Proximity_sensor2",
        "Proximity_sensor3",
        "Proximity_sensor4",
        "Proximity_sensor5",
        "Proximity_sensor6",
        "Proximity_sensor7",
        "Proximity_sensor8",
        "Proximity_sensor9",
        "Proximity_sensor10",
        "Proximity_sensor11",
        "Proximity_sensor12",
        "Proximity_sensor13",
        "Proximity_sensor14",
        "Proximity_sensor15",
        "Proximity_sensor16",
        "Proximity_sensor17",
        "Proximity_sensor18",
        "Proximity_sensor19",
        "Proximity_sensor20",
        ]

    OBJ_NAMES = INDY_OBJ_NAMES + SENSOR_OBJ_NAMES

    def __init__(self):
        # vrep client variables
        self.do_next_step = True

        # connect to vrep server
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_V-REP', 'b0RemoteApi')

        # run some command before the synchronous mode
        self.get_object_handles()

        # set simualtion synchronous mode
        self.client.simxSynchronous(True)

        # vrep robot bridge
        self.indy7 = VrepRobotStateBridge(self.client, self.obj_handles, self.INDY_OBJ_NAMES)

        # vrep distance sensor bridge
        self.distance_sensor = VrepDistanceSensorBridge(self.client, self.obj_handles, self.SENSOR_OBJ_NAMES)
        self.distance_sensor.get_rel_sensor_transformation("joint5")

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulation_step_done_cb))

        # # init ros node
        # rospy.init_node('vrep_interface')

        # # ros pub
        # self.irb120_joint_states_pub = rospy.Publisher(
        #     '/irb120_arm_controller/joint_states',
        #     JointState,
        #     queue_size=1,
        #     )
        # self.irb120_control_state_pub = rospy.Publisher(
        #     '/irb120_arm_controller/feedback_states',
        #     FollowJointTrajectoryFeedback,
        #     queue_size=1,
        #     )
        # self.indy7_joint_states_pub = rospy.Publisher(
        #     '/indy7_arm_controller/joint_states',
        #     JointState,
        #     queue_size=1,
        #     )
        # self.indy7_control_state_pub = rospy.Publisher(
        #     '/indy7_arm_controller/feedback_states',
        #     FollowJointTrajectoryFeedback,
        #     queue_size=1,
        #     )
        # self.rgb_image_pub = rospy.Publisher(
        #     '/rgb_image',
        #     Image,
        #     queue_size=1
        #     )
        # self.depth_image_pub = rospy.Publisher(
        #     '/depth_image',
        #     Image,
        #     queue_size=1
        #     )

        # # ros sub
        # self.irb120_joint_pathcommand = rospy.Subscriber(
        #     '/irb120_arm_controller/joint_path_command',
        #     JointTrajectory,
        #     self.irb120.ros_joint_path_cb,
        #     )
        # self.indy7_joint_pathcommand = rospy.Subscriber(
        #     '/indy7_arm_controller/joint_path_command',
        #     JointTrajectory,
        #     self.indy7.ros_joint_path_cb,
        #     )

    def get_object_handles(self):
        self.obj_handles = dict()
        for name in self.OBJ_NAMES:
            self.obj_handles[name] = self.client.simxGetObjectHandle(
                name,
                self.client.simxServiceCall()
                )[1]

    def simulation_step_start_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        # print(self.distance_sensor.distance)
        self.indy7.excute_trajectory()

    def simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        # # after step done publish ros msgs
        # self.irb120_joint_states_pub.publish(self.irb120.joint_states_msg)
        # self.irb120_control_state_pub.publish(self.irb120.control_states_msg)
        # self.indy7_joint_states_pub.publish(self.indy7.joint_states_msg)
        # self.indy7_control_state_pub.publish(self.indy7.control_states_msg)
        # self.rgb_image_pub.publish(self.rgb_camera.image_msg)
        # self.depth_image_pub.publish(self.depth_camera.image_msg)

        # change do_next_step state
        self.do_next_step = True

    def start_simulation(self):
        self.client.simxStartSimulation(self.client.simxDefaultPublisher())
        print("start vrep simulation with bluezero remote API")

    def stop_simulation(self):
        self.client.simxStopSimulation(self.client.simxDefaultPublisher())
        print("stop simulation")

    def step_simulation(self):
        while not self.do_next_step:
            self.client.simxSpinOnce()
        self.do_next_step = False
        self.client.simxSynchronousTrigger()


if __name__ == '__main__':
    vrep = VrepInterface()
    vrep.start_simulation()

    startTime = time.time()
    while time.time() < startTime + 3:
        vrep.step_simulation()

    vrep.stop_simulation()
