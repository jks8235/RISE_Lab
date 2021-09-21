import numpy as np
import rospy
import struct
import math
import time
import pandas as pd
from scipy.spatial.transform import Rotation as R
import tf
import copy

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray

class PointCloud_Shape:
    def __init__(self, points, pose):
        self.br = tf.TransformBroadcaster()
        self.header = Header()
        self.pointcloud_publisher = rospy.Publisher('test/pcl', PointCloud2, queue_size=10)
        
        self.points = points
        self.pose = pose

        self.bundle_transformation_pub = rospy.Publisher(
            'bundle_transformation_data',
            Float32MultiArray,
            queue_size=2
        )

        self.sensor_tf = [[0.0, 0.0, 0.0, 0.0, -0.0726290, 0.0, 0.9973590],
                          [0.0, 0.0, 0.0, 0.0363145, -0.0628985, 0.4986795, 0.8637382],
                          [0.0, 0.0, 0.0, 0.0628985, -0.0363145, 0.8637383, 0.4986795],
                          [0.0, 0.0, 0.0, 0.0726289, -0.0, 0.9973590, 0.0],
                          [0.0, 0.0, 0.0, 0.0628985, 0.0363145, 0.8637383, -0.4986796],
                          [0.0, 0.0, 0.0, 0.0363145, 0.0628985, 0.4986795, -0.8637382],
                          [0.0, 0.0, 0.0, -0.0, -0.0485011, 0.0, 0.9988232],
                          [0.0, 0.0, 0.0, 0.0242506, -0.0420032, 0.4994116, 0.8650061],
                          [0.0, 0.0, 0.0, 0.0420032, -0.0242505, 0.8650061, 0.4994116],
                          [0.0, 0.0, 0.0, 0.0485012, 0.0, 0.9988231, 0.0],
                          [0.0, 0.0, 0.0, 0.0420032, 0.0242506, 0.8650063, -0.4994116],
                          [0.0, 0.0, 0.0, 0.0242506, 0.0420032, 0.4994116, -0.8650061],
                          [0.0, 0.0, 0.0, 0.0, -0.0322830, 0.0, 0.9994788],
                          [0.0, 0.0, 0.0, 0.0161415, -0.0279578, 0.4997394, 0.8655741],
                          [0.0, 0.0, 0.0, 0.0279578, -0.0161415, 0.8655740, 0.4997394],
                          [0.0, 0.0, 0.0, 0.0322830, 0.0, 0.9994788, 0.0],
                          [0.0, 0.0, 0.0, 0.0279578, 0.0161415, 0.8655740, -0.4997394],
                          [0.0, 0.0, 0.0, 0.0161415, 0.0279579, 0.4997394, -0.8655741],
                          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]

        self.sensor_names = ["Proximity_sensor_1_1",
                             "Proximity_sensor_1_2",
                             "Proximity_sensor_1_3",
                             "Proximity_sensor_1_4",
                             "Proximity_sensor_1_5",
                             "Proximity_sensor_1_6",
                             "Proximity_sensor_2_1",
                             "Proximity_sensor_2_2",
                             "Proximity_sensor_2_3",
                             "Proximity_sensor_2_4",
                             "Proximity_sensor_2_5",
                             "Proximity_sensor_2_6",
                             "Proximity_sensor_3_1",
                             "Proximity_sensor_3_2",
                             "Proximity_sensor_3_3",
                             "Proximity_sensor_3_4",
                             "Proximity_sensor_3_5",
                             "Proximity_sensor_3_6",
                             "Proximity_sensor_Center"]

    def get_point_cloud(self, points):
        header = self.header
        header.stamp = rospy.Time.now()
        header.frame_id = "sensor"

        fields = [PointField('x', 0, PointField.FLOAT32,1),
                  PointField('y', 4, PointField.FLOAT32,1),
                  PointField('z', 8, PointField.FLOAT32,1),
                  PointField('intensity', 12, PointField.FLOAT32,1)]

        pcl = point_cloud2.create_cloud(header, fields, points)
        return pcl

    def get_tf_point_cloud_data(self, full_quarternion_bundle, point_cloud_data):

        pcl_transform = geometry_msgs.msg.TransformStamped()
        sensor_to_point_cloud = []
        
        point_cloud = self.get_point_cloud(point_cloud_data)

        pcl_transform.header = self.header
        pcl_transform.header.stamp = rospy.Time.now()
        pcl_transform.header.frame_id = "sensor_bundle"
        pcl_transform.child_frame_id = "sensor"
        pcl_transform.transform.translation.x = full_quarternion_bundle[0]
        pcl_transform.transform.translation.y = full_quarternion_bundle[1]
        pcl_transform.transform.translation.z = full_quarternion_bundle[2]
        pcl_transform.transform.rotation.x = full_quarternion_bundle[3]
        pcl_transform.transform.rotation.y = full_quarternion_bundle[4]
        pcl_transform.transform.rotation.z = full_quarternion_bundle[5]
        pcl_transform.transform.rotation.w = full_quarternion_bundle[6]
        
        '''
        do_transform_cloud
        @First param: point cloud data
        @Second param: The Transform message to convert.
        @Second param type: geometry_msgs.msg.TransformStamped
        '''
        cloud_out = do_transform_cloud(point_cloud, pcl_transform)
        sensor_to_point_cloud.append(cloud_out)

        return sensor_to_point_cloud

    def make_point(self):

        sensor_num = 19

        for point_num in range(self.points.shape[1]):
            point_cloud_data = []

            for sensor in range(sensor_num):

                data = self.points[sensor, point_num]
                
                if  0.75 >= data and data >= 0.01:
                    intensity = 0.1
                
                    full_quarternion_sensor = self.sensor_tf[sensor]
                    point = [0.0, 0.0, data]

                    translation = full_quarternion_sensor[:3]
                    quarternion = full_quarternion_sensor[3:]

                    rot_matrix = R.as_dcm(R.from_quat(quarternion))

                    world_point = (np.dot(rot_matrix, point) + translation).tolist()
                    world_point.append(intensity) 

                    point_cloud_data.append(world_point)

            full_quarternion_bundle = self.pose[126:133, point_num]
            self.bundle_transformation_msg = Float32MultiArray()
            self.bundle_transformation_msg.data = full_quarternion_bundle
            self.bundle_transformation_pub.publish(self.bundle_transformation_msg)

            pcl = self.get_tf_point_cloud_data([0, 0, 0, 0, 0, 0, 1], point_cloud_data)

            if len(pcl) != 0:
                transform_pc = copy.copy(pcl[0])
                for pc in pcl[1:]:
                    transform_pc.width += pc.width
                    transform_pc.data += pc.data
    
            self.pointcloud_publisher.publish(transform_pc)
            time.sleep(0.001)

            print("Making", point_num, self.points.shape[1])
        
        print("Making Done")

def load_data(mode='test'):
    points = None
    pose = None

    if mode == 'test':
        for i in range(1):
            path1 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/test_1/output_Fold_%d.csv'%(i+1)
            path2 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/test_1/pose_Fold_%d.csv'%(i+1)
            temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
            temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

            points = merge_data(points, temp_points, axis=1)
            pose = merge_data(pose, temp_pose, axis=1)

        points = points.T


    elif mode == 'predict':
        for i in range(1):
            path2 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/test_1/pose_Fold_%d.csv'%(i+1)
            temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

            pose = merge_data(pose, temp_pose, axis=1)

        path1 = '/home/jee/work_space/data_folder/Sensor_learning/data/hexagon/result/learning1_Fold1/predict_Fold_1_1.csv'
        points = np.array(pd.read_csv(path1, sep=",", header=None))

    pose = pose.T
    points = points.T

    print('Data Load Done')
    print(points.shape)
    print(pose.shape)

    return points, pose

def merge_data(total_data, add_data, axis=0):
    total = total_data

    if type(total)==type(None):
        total = add_data
#         print('merge ok')
    else:
        total = np.concatenate((total, add_data),axis)
    
    total = pd.DataFrame(total)
    total = np.array(total)

    return total

if __name__ == '__main__':

    rospy.init_node('make_point_cloud')
    print("start")

    points, pose = load_data(mode='predict')

    point_cloud_vrep = PointCloud_Shape(points, pose)

    count = 0

    while not rospy.is_shutdown():
        try:
            if count < 1:
                count += 1 
                point_cloud_vrep.make_point()

        except rospy.ROSInterruptException:
            pass