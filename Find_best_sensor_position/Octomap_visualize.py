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

        for point_num in range(self.points.shape[0]):

            for sensor_num in range(4):

                distance = self.points[point_num, sensor_num]
            
                if  distance < 0.15:

                    senor_model_points = self._make_sensor_model_pcl(distance)

                    full_quarternion_bundle = self.pose[point_num, sensor_num*7:(sensor_num+1)*7]
                    self.bundle_transformation_msg = Float32MultiArray()
                    self.bundle_transformation_msg.data = full_quarternion_bundle
                    self.bundle_transformation_pub.publish(self.bundle_transformation_msg)

                    pcl = self.get_tf_point_cloud_data([0, 0, 0, 0, 0, 0, 1], senor_model_points)

                    if len(pcl) != 0:
                        transform_pc = copy.copy(pcl[0])
                        for pc in pcl[1:]:
                            transform_pc.width += pc.width
                            transform_pc.data += pc.data
            
                    self.pointcloud_publisher.publish(transform_pc)
            
            # else:
                # print("No data", self.points[point_num, 1])

            time.sleep(0.001)

            print("Making", point_num, self.points.shape[1])
        
        print("Making Done")

    def _make_sensor_model_pcl(self, distance, angle_resol = 0.1):
        intensity = 0.1
        angle_limit = 12.5
        
        points = []

        bow_angles = [i*angle_resol for i in range(int(angle_limit/angle_resol) + 1)]
        round_angles = [i*15 for i in range(int(360/15) + 1)]

        for bow_angle in bow_angles:
            z = distance*math.cos(deg2rad(bow_angle))
            r = distance*math.sin(deg2rad(bow_angle))
            for round_angle in round_angles:
                x = r*math.cos(deg2rad(round_angle))
                y = r*math.sin(deg2rad(round_angle))

                points += [[x, y, z, intensity]]

        return points

def load_data():
    points = None
    pose = None

    for i in range(1):
        path1 = '/media/jee/FC12-B7D8/data_folder/Best_sensor_position/No_obstacle_distance_data.npy'
        path2 = '/media/jee/FC12-B7D8/data_folder/Best_sensor_position/No_obstacle_sensor_pose.npy'
       
        temp_points = np.load(path1)
        temp_pose = np.load(path2)

        points = merge_data(points, temp_points, axis=1)
        pose = merge_data(pose, temp_pose, axis=1)

    print(points.shape, pose.shape)

    print('Data Load Done')

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

def deg2rad(deg):
    rad = deg/180*math.pi

    return rad

if __name__ == '__main__':

    rospy.init_node('make_point_cloud')
    print("start")

    points, pose = load_data()

    point_cloud_vrep = PointCloud_Shape(points, pose)

    count = 0

    while not rospy.is_shutdown():
        try:
            if count < 1:
                count += 1 
                point_cloud_vrep.make_point()

        except rospy.ROSInterruptException:
            pass