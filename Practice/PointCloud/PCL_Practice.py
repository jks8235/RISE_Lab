#!/usr/bin/env python
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import rospy
import struct
import time

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

points = []
points2 = []
lim = 16
for i in range(lim):
    for j in range(lim):
        for k in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(k) / lim
            r = int(x * 255.0)
            g = int(y * 255.0)
            b = int(z * 255.0)
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            pt = [x, y, z, rgb]
            pt2 = [x, y, z]

            points.append(pt)
            points2.append(pt2)

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]

def pcl_pub_xyzrgb(fields,points):
    header = Header()
    header.stamp = rospy.Time.now()
    # header.frame_id = '{}'.format(self.sensor_obj)
    header.frame_id = 'map'
    pcl = point_cloud2.create_cloud(header, fields, points)
    # pointcloud_publisher = rospy.Publisher("{}".format(self.sensor_obj), PointCloud2, queue_size=10)
    pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
    pointcloud_publisher.publish(pcl)
    time.sleep(0.5)

def pcl_pub_xyz(points):
    header = Header()
    header.stamp = rospy.Time.now()
    # header.frame_id = '{}'.format(self.sensor_obj)
    header.frame_id = 'map'
    pcl = point_cloud2.create_cloud_xyz32(header, points)
    # pointcloud_publisher = rospy.Publisher("{}".format(self.sensor_obj), PointCloud2, queue_size=10)
    pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
    pointcloud_publisher.publish(pcl)
    time.sleep(0.5)

rospy.init_node('make_point_cloud')
while not rospy.is_shutdown():
    try:
#        pcl_pub_xyzrgb(fields, points)
        pcl_pub_xyz(points2)
    except rospy.ROSInterruptException:
        pass
