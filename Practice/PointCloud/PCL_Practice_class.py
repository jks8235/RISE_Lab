#!/usr/bin/env python
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import numpy as np
import rospy
import struct
import math
import time

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

### Base Calss set : publish point, publish point & rgb
class PointCloud_Shape:
    def __init__(self):
        self.result = 0

    def pcl_pub_xyzrgb(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
        ]

        pcl = point_cloud2.create_cloud(header, fields, points)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)
        
    def pcl_pub_xyz(self,points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl = point_cloud2.create_cloud_xyz32(header, points)
        pointcloud_publisher = rospy.Publisher('test_pcl', PointCloud2, queue_size=10)
        pointcloud_publisher.publish(pcl)
        time.sleep(0.5)

### Cube ###
class PointCloud_Shape_Cube(PointCloud_Shape):
    def __init__(self):
        self.result = 0

    def make_point(self):
        Point_xyz = []
        lim = 16
        for i in range(lim):
            for j in range(lim):
                for k in range(lim):
                    x = float(i) / 4
                    y = float(j) / 4
                    z = float(k) / 4
                    
                    Pt_xyz = [x, y, z]

                    Point_xyz.append(Pt_xyz)
        time.sleep(0.5)
        self.pcl_pub_xyz(Point_xyz)

### Color Cube ###
class PointCloud_Shape_ColorCube(PointCloud_Shape):
    def __init__(self):
        self.result = 0

    def make_point(self):
        Point_xyzrgb = []
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

                    Pt_xyzrgb = [x, y, z, rgb]

                    Point_xyzrgb.append(Pt_xyzrgb)
        time.sleep(0.5)
        self.pcl_pub_xyzrgb(Point_xyzrgb)

### part of sphere - vertical, horizontal
class Sphere_part(PointCloud_Shape):
    def __init__(self, angle_x, angle_y, distance):
        self.angle_x = angle_x
        self.angle_y = angle_y
        self.distance = distance
        self.angle_resolution = 1.0
        self.angle_range = 1 + (int(max(self.angle_x,self.angle_y)/self.angle_resolution))/2
        self.x_limit = self.distance*math.sin(np.deg2rad(self.angle_x/2))
        self.y_limit = self.distance*math.sin(np.deg2rad(self.angle_y/2))

    def make_point(self):
        angle=0.0
        Point_xyz=[]

        for n in range(0,self.angle_range):
            angle = self.angle_resolution * n
            
            s = math.sin(np.deg2rad(float(angle)))
            c = math.cos(np.deg2rad(float(angle)))

            z = self.distance*c
            xy_radius = self.distance*s
            
            for r in range(0,int(360/self.angle_resolution)+1):

                angle_rotation = r*self.angle_resolution

                x = xy_radius*math.cos(np.deg2rad(angle_rotation))
                y = xy_radius*math.sin(np.deg2rad(angle_rotation))
                                
                if abs(x) <= abs(self.x_limit) and abs(y) <= abs(self.y_limit):
                    Pt=[x, y, z]
                    Point_xyz.append(Pt)
        time.sleep(0.5)
        self.pcl_pub_xyz(Point_xyz)

### Ellipse
class Ellipse(PointCloud_Shape):
    def __init__(self, angle_x, angle_y, distance):
        self.angle_x = angle_x
        self.angle_y = angle_y
        self.distance = distance
        self.angle_resolution = 1.0
        self.angle_range = 1 + (int(max(self.angle_x,self.angle_y)/self.angle_resolution))/2
        self.x_limit = self.distance*math.sin(np.deg2rad(self.angle_x/2))
        self.y_limit = self.distance*math.sin(np.deg2rad(self.angle_y/2))

    def make_point(self):
        angle=0.0
        Point_xyz=[]

        for n in range(0,self.angle_range):
            angle = self.angle_resolution * n
            
            s = math.sin(np.deg2rad(float(angle)))
            c = math.cos(np.deg2rad(float(angle)))

            z = self.distance*c
            x_limit = self.distance*math.sin(np.deg2rad(self.angle_x/2*n/self.angle_range))
            y_limit = self.distance*math.sin(np.deg2rad(self.angle_y/2*n/self.angle_range))

            for r in range(0,int(360/self.angle_resolution)+1):

                angle_rotation = r*self.angle_resolution

                x = x_limit*math.cos(np.deg2rad(angle_rotation))
                y = y_limit*math.sin(np.deg2rad(angle_rotation))
                
                Pt=[x, y, z]
                Point_xyz.append(Pt)
        time.sleep(0.5)
        self.pcl_pub_xyz(Point_xyz)

### Main Code ###
rospy.init_node('make_point_cloud')
Color_cube = PointCloud_Shape_ColorCube()
Cube = PointCloud_Shape_Cube()
Sphere_part = Sphere_part(120, 60, 1)
Ellip = Ellipse(120,60,1)

while not rospy.is_shutdown():
    try:
        Cube.make_point()
        #Color_cube.make_point()
        #Sphere_part.make_point()
        #Ellip.make_point()

    except rospy.ROSInterruptException:
        pass
