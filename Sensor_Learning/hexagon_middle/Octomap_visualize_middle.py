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

        # hex modle
        self.sensor_tf = [
        [0, 0, 0, 0.0, 0.043619390577077866, 0.0, 0.9990482330322266],
        [0, 0, 0, -0.021809710189700127, 0.03777552396059036, 0.4995241165161133, 0.8652011156082153],
        [0, 0, 0, -0.03777549788355827, 0.021809730678796768, 0.8652011156082153, 0.4995241165161133],
        [0, 0, 0, -0.043619412928819656, 0.0, 0.9990482330322266, 0.0],
        [0, 0, 0, -0.037775490432977676, -0.02180967852473259, 0.8652011752128601, -0.49952417612075806],
        [0, 0, 0, -0.021809721365571022, -0.037775445729494095, 0.49952420592308044, -0.8652011752128601],
        [0, 0, 0, 0, 0.08715569972991943, 0.0, 0.9961947798728943],
        [0, 0, 0, -0.021150950342416763, 0.07893654704093933, 0.25795334577560425, 0.9626950621604919],
        [0, 0, 0, -0.0435778982937336, 0.07547909766435623, 0.49809736013412476, 0.8627299070358276],
        [0, 0, 0, -0.05778554826974869, 0.057785507291555405, 0.7047416567802429, 0.7047417163848877],
        [0, 0, 0, -0.07547914236783981, 0.04357782378792763, 0.8627299070358276, 0.49809733033180237],
        [0, 0, 0, -0.07893652468919754, 0.02115095593035221, 0.9626951217651367, 0.25795334577560425],
        [0, 0, 0, -0.08715571463108063, 7.275956746821688e-12, 0.9961947202682495, 0],
        [0, 0, 0, -0.07893649488687515, -0.021150965243577957, 0.9626950025558472, -0.257953405380249],
        [0, 0, 0, -0.07547907531261444, -0.04357790946960449, 0.8627299070358276, -0.49809733033180237],
        [0, 0, 0, -0.057785481214523315, -0.057785533368587494, 0.7047415971755981, -0.7047417759895325],
        [0, 0, 0, -0.04357782006263733, -0.07547906786203384, 0.49809741973876953, -0.8627299666404724],
        [0, 0, 0, -0.021150965243577957, -0.07893647253513336, 0.2579532563686371, -0.9626950621604919],
        [0, 0, 0, 0, 0, 0, 1]
        ]

        print(len(self.sensor_tf))

        self.sensor_names = [
        "Proximity_sensor_Center",
        "Proximity_sensor_1_1",
        "Proximity_sensor_1_2",
        "Proximity_sensor_1_3",
        "Proximity_sensor_1_4",
        "Proximity_sensor_1_5",
        "Proximity_sensor_1_6",
        "Proximity_sensor_2_01",
        "Proximity_sensor_2_02",
        "Proximity_sensor_2_03",
        "Proximity_sensor_2_04",
        "Proximity_sensor_2_05",
        "Proximity_sensor_2_06",
        "Proximity_sensor_2_07",
        "Proximity_sensor_2_08",
        "Proximity_sensor_2_09",
        "Proximity_sensor_2_10",
        "Proximity_sensor_2_11",
        "Proximity_sensor_2_12"
        ]

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

        for point_num in range(self.points.shape[0]):
            point_cloud_data = []

            for sensor in range(sensor_num):

                data = self.points[point_num, sensor]
                
                if  0.65 >= data and data >= 0.02:
                    intensity = 0.1
                
                    full_quarternion_sensor = self.sensor_tf[sensor]
                    point = [0.0, 0.0, data]

                    translation = full_quarternion_sensor[:3]
                    quarternion = full_quarternion_sensor[3:]

                    rot_matrix = R.as_dcm(R.from_quat(quarternion))

                    world_point = (np.dot(rot_matrix, point) + translation).tolist()
                    world_point.append(intensity) 

                    point_cloud_data.append(world_point)

            full_quarternion_bundle = self.pose[point_num, 160:167]
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
            # path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/PAP_test_1/output_Fold_1.npy'
            # path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/PAP_test_1/pose_Fold_1.npy'
            path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/test_3/output_Fold_1.npy'
            path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/test_3/input_Fold_1.npy'

            temp_points = np.load(path1)
            temp_pose = np.load(path2)

            # temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
            # temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

            points = merge_data(points, temp_points, axis=1)
            pose = merge_data(pose, temp_pose, axis=1)

        print(points.shape, pose.shape)

    elif mode == 'predict':
        for i in range(1):
            # path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/result/predict_Fold_PAP_test_1.csv'
            # path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/PAP_test_1/pose_Fold_1.npy'
            path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning_ver2/data/hexagon_middle/result/predict_Fold_test_2.csv'
            path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/test_2/input_Fold_1.npy'

            # temp_points = np.road(path1))
            temp_pose = np.load(path2)

            temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
            # temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

            points = merge_data(points, temp_points, axis=1)
            pose = merge_data(pose, temp_pose, axis=1)


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