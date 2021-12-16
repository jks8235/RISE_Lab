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
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.031157810240983963, 0.0, 0.0, 0.9995144605636597],
        [0.0, 0.0, 0.0, 0.0269834715873003, 0.01557888463139534, 0.49975723028182983, 0.8656049370765686],
        [0.0, 0.0, 0.0, 0.015578911639750004, 0.026983456686139107, 0.8656049966812134, 0.4997572898864746],
        [0.0, 0.0, 0.0, 0.0, 0.031157812103629112, 0.9995144605636597, 0.0],
        [0.0, 0.0, 0.0, -0.015578892081975937, 0.026983439922332764, 0.8656049966812134, -0.4997572898864746],
        [0.0, 0.0, 0.0, -0.026983434334397316, 0.015578964725136757, 0.49975723028182983, -0.8656049370765686],
        [0.0, 0.0, 0.0, 0.05395681783556938, 0.0, 0.0, 0.9985432624816895],
        [0.0, 0.0, 0.0, 0.0601714625954628, 0.016122883185744286, 0.25831636786460876, 0.9640498757362366],
        [0.0, 0.0, 0.0, 0.0467279814183712, 0.02697833999991417, 0.4992716908454895, 0.8647638559341431],
        [0.0, 0.0, 0.0, 0.044048529118299484, 0.04404854774475098, 0.7057334780693054, 0.7057334780693054],
        [0.0, 0.0, 0.0, 0.02697843685746193, 0.04672793298959732, 0.8647638559341431, 0.4992716610431671],
        [0.0, 0.0, 0.0, 0.0161229707300663, 0.06017143279314041, 0.9640498161315918, 0.25831639766693115],
        [0.0, 0.0, 0.0, 0.0, 0.05395681783556938, 0.9985432624816895, 0.0],
        [0.0, 0.0, 0.0, -0.01612282544374466, 0.0601714551448822, 0.9640498161315918, -0.25831636786460876],
        [0.0, 0.0, 0.0, -0.026978440582752228, 0.04672793298959732, 0.8647639155387878, -0.4992716312408447],
        [0.0, 0.0, 0.0, -0.04404858127236366, 0.044048577547073364, 0.7057335376739502, -0.7057334780693054],
        [0.0, 0.0, 0.0, -0.046728022396564484, 0.02697840891778469, 0.4992716312408447, -0.8647637963294983],
        [0.0, 0.0, 0.0, -0.060171447694301605, 0.01612294651567936, 0.2583164572715759, -0.9640498161315918],
        [0.0, 0.0, 0.0, 0.09336113184690475, 0.0, 0.0, 0.9956322908401489],
        [0.0, 0.0, 0.0, 0.08935834467411041, 0.01575624942779541, 0.17293186485767365, 0.9807453155517578],
        [0.0, 0.0, 0.0, 0.08526477217674255, 0.031033819541335106, 0.3406093120574951, 0.9358163475990295],
        [0.0, 0.0, 0.0, 0.08085310459136963, 0.04668052867054939, 0.49781617522239685, 0.8622428774833679],
        [0.0, 0.0, 0.0, 0.06950842589139938, 0.05832449346780777, 0.6401361227035522, 0.7628844380378723],
        [0.0, 0.0, 0.0, 0.058324526995420456, 0.06950847804546356, 0.7628844976425171, 0.6401360034942627],
        [0.0, 0.0, 0.0, 0.04668063670396805, 0.08085312694311142, 0.8622429370880127, 0.49781620502471924],
        [0.0, 0.0, 0.0, 0.031033793464303017, 0.08526476472616196, 0.9358164072036743, 0.34060925245285034],
        [0.0, 0.0, 0.0, 0.015756327658891678, 0.0893583819270134, 0.9807453155517578, 0.17293182015419006],
        [0.0, 0.0, 0.0, 0.0, 0.09336113184690475, 0.9956322908401489, 0.0],
        [0.0, 0.0, 0.0, -0.015756281092762947, 0.089358389377594, 0.9807453155517578, -0.1729319840669632],
        [0.0, 0.0, 0.0, -0.03103383630514145, 0.08526474982500076, 0.9358162879943848, -0.3406093418598175],
        [0.0, 0.0, 0.0, -0.04668061435222626, 0.08085311204195023, 0.8622428178787231, -0.49781617522239685],
        [0.0, 0.0, 0.0, -0.05832457169890404, 0.06950849294662476, 0.7628843784332275, -0.6401361227035522],
        [0.0, 0.0, 0.0, -0.06950844824314117, 0.05832454934716225, 0.6401360034942627, -0.7628844976425171],
        [0.0, 0.0, 0.0, -0.08085310459136963, 0.04668058454990387, 0.49781620502471924, -0.8622428774833679],
        [0.0, 0.0, 0.0, -0.08526475727558136, 0.031033821403980255, 0.3406091630458832, -0.9358164072036743],
        [0.0, 0.0, 0.0, 0.0893583595752716, -0.015756314620375633, -0.17293187975883484, 0.9807453751564026]]

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
        "Proximity_sensor_2_12",
        "Proximity_sensor_3_01",
        "Proximity_sensor_3_02",
        "Proximity_sensor_3_03",
        "Proximity_sensor_3_04",
        "Proximity_sensor_3_05",
        "Proximity_sensor_3_06",
        "Proximity_sensor_3_07",
        "Proximity_sensor_3_08",
        "Proximity_sensor_3_09",
        "Proximity_sensor_3_10",
        "Proximity_sensor_3_11",
        "Proximity_sensor_3_12",
        "Proximity_sensor_3_13",
        "Proximity_sensor_3_14",
        "Proximity_sensor_3_15",
        "Proximity_sensor_3_16",
        "Proximity_sensor_3_17",
        "Proximity_sensor_3_18",
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

        sensor_num = 37

        for point_num in range(self.points.shape[0]):
            point_cloud_data = []

            for sensor in range(sensor_num):

                data = self.points[point_num, sensor]
                
                if  0.7 >= data and data >= 0.02:
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
            path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/test_1/output_Fold_%d.npy'%(i+1)
            path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/test_1/input_Fold_%d.npy'%(i+1)

            temp_points = np.load(path1)
            temp_pose = np.load(path2)

            # temp_points = np.array(pd.read_csv(path1, sep=",", header=None))
            # temp_pose = np.array(pd.read_csv(path2, sep=",", header=None))

            points = merge_data(points, temp_points, axis=1)
            pose = merge_data(pose, temp_pose, axis=1)

        print(points.shape, pose.shape)

    elif mode == 'predict':
        for i in range(1):
            path1 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/result/predict_Fold_%d_total_data_1.csv'%(i+1)
            path2 = '/media/jee/FC12-B7D8/data_folder/Sensor_Learning/data/hexagon_middle/test_1/input_Fold_%d.npy'%(i+1)

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