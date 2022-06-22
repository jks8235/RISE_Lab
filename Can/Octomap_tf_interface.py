#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Jin-Jae Shin (RISE)

import rospy
import tf
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Float32MultiArray
import copy

def divide_list(l, n): 
    # 리스트 l의 길이가 n이면 계속 반복
    for i in range(0, len(l), n): 
        yield l[i:i + n] 

def e2q(value):
        """
        Convenience method euler to quaternion
        @param: value A float, euler angle
        """
        return tf.transformations.quaternion_from_euler(value[0],value[1],value[2])

class TransformationInterface:
    def __init__(self):
        '''
        Transformation information update 
        @param : link   String list = tf tree link
        '''
        rospy.init_node('transformation_interface')

        self.link = "world"
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber('bundle_transformation_data',
                        Float32MultiArray,
                        self.set_bundle_tf_cb)

        self.bundle_tf = [0, 0, 0, 0, 0, 0, 1]

    def set_bundle_tf_cb(self,msg):
        '''
        rel data가 들어있는 msg를 받아 7개씩 끊어 읽어 서로의 관계를 정의한다.
        @param : Float32MultiArray  rel_data = [x,y,z,w_x,w_y,w_z,w]
        '''
        sensor_rel_data = list(msg.data)
        self.bundle_tf = copy.copy(sensor_rel_data)
        
    def set_tf(self):
        self.br.sendTransform(self.bundle_tf[0:3],
                              self.bundle_tf[3:],
                              rospy.Time.now(),
                              "sensor_bundle",
                              "world")

if __name__ == "__main__":
    TI = TransformationInterface()
    
    while not rospy.is_shutdown():
        TI.set_tf()
        