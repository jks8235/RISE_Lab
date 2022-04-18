
import numpy as np
from numpy import meshgrid
from numpy import linspace
from scipy.linalg import norm
from Forward_Kinematics import t_mat
from Forward_Kinematics import FKIndy7 as FK

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Colormap

class  SelfRecognition:

    def __init__(self, joint_states, sensor_config, show = False):
        """
        Input:
        @param list joint_states: joint positions of robot [j0, j1, ..., jn]

        @param dictionary sensor_config: dictionary for sensor configuration

        sensor_config = {
        'DH' = [alpha, theta, a ,d]
        'parent_link' = 'jointnum'  (Ex. 'joint0' or 'joint5')
        'sensor_mesh' = [X, Y, Z]
        }

        @param boolean show (default = False): If Matplotlib representation is needed set as True
        """

        self.joint_states = joint_states
        self.sensor_config = sensor_config
        self.indy = FK(joint_states)
        self.show = show

        self.sensors_detected = []

    def update_joint_states(self, joint_states):
        """
        Update joint configurations of the Indy7

        Input:
        @param list joint_states: joint positions of robot
        """
        self.indy.update_joints(joint_states)

    def update_sensor_config(self, sensor_config):
        """
        Update sensor configuration dictionary

        Input:
        @param dictionary sensor_config
        """
        self.sensor_config = sensor_config

    def self_collision_check(self):
        """
        Checks if a link is within the boundary of the sensor that detects an object.

        Output:
        @param boolean self_recognition : returns true if sensor boundary overlaps link
        """

        if self.show is True:
            fig = matplotlib.pyplot.figure()
            self.ax  = fig.add_subplot(111, projection = '3d')
            self.ax.set_xlabel('x (m)')
            self.ax.set_ylabel('y (m)')
            self.ax.set_zlabel('z (m)')
            self.ax.set_aspect('equal')
            self.ax.set_xlim([-1, 1])
            self.ax.set_ylim([-1, 1])
            self._create_link_mesh()

        self._main()

        if self.show is True:
            matplotlib.pyplot.show()

        return self.self_recognition

    def _main(self):
        
        sensor_mesh = self.sensor_config['sensor_mesh']
        robot_joint_positions = self.indy.get_DH_positions()
        robot_link_radius = self.indy.get_link_radius()
        self.self_recognition = False

        for link in xrange(len(robot_link_radius)):
            
            frame_0 =  robot_joint_positions[0]
            robot_joint_positions = np.delete(robot_joint_positions, 0, 0)
            frame_1 =  robot_joint_positions[0]

            for i in xrange(len(sensor_mesh[0])): 
                for j in xrange(len(sensor_mesh[0][0])): 

                    sensor_mesh_point = self._transform_mesh_point([sensor_mesh[0][i][j], sensor_mesh[1][i][j], sensor_mesh[2][i][j]], self.sensor_config['DH'], self.sensor_config['parent_link'])
                    distance_from_link = self._lineseg_dist(sensor_mesh_point, frame_0, frame_1)

                    if self.show is True:
                        self.ax.scatter(sensor_mesh_point[0], sensor_mesh_point[1], sensor_mesh_point[2], color = 'r')
                    
                    if distance_from_link < robot_link_radius['link{}'.format(link)]:
                        # print('Link {} is within sensor'.format(link))
                        self.self_recognition = True
                        

    def _lineseg_dist(self, p, a, b):

        """
        Calculates the distance between a point and a segment

        @param nparray p: Position of Point in 3D Cartesion Space [x, y, z]
        @param nparray a: Start Position of Segment in 3D Cartesian Space [x, y, z]
        @param nparray b: End Position of Segment in 3D Cartesian Space [x, y, z]

        Output
        return : int : Distance between point and segment (m)
        """

        # normalized tangent vector
        d = np.divide(b - a, np.linalg.norm(b - a))

        # signed parallel distance component
        s = np.dot(a - p, d)
        t = np.dot(p - b, d)

        # clamped parallel distance
        h = np.maximum.reduce([s, t, 0])

        # perpendicular distance component
        c = np.cross(p - a, d)

        return np.hypot(h, np.linalg.norm(c))

    def _transform_mesh_point(self, point, DH_tran, parent_link):
        """
        Transforms Sensor Mesh Location Based on Parent "Joint" (Ex. parent_link = 'joint0')

        Input:
        @param list point: cartesian position of mesh point [x,y,z]
        @param list DH_tran: DH parameters for transformation from parent link [alpha, theta, a, d]
        @param string parent_link: joint name where the DH transformation takes place 
        """

        point_tran_mat = np.array([ [1.0, 0.0, 0.0, point[0]],
                                    [0.0, 1.0, 0.0, point[1]],
                                    [0.0, 0.0, 1.0, point[2]],
                                    [0.0, 0.0, 0.0, 1.0] ])
        
        transformed_point_mat = np.matmul(self.indy.get_tran_matrix_at_joint(parent_link), np.matmul(point_tran_mat, t_mat(DH_tran[0], DH_tran[1] ,DH_tran[2], DH_tran[3])))

        transformed_point = np.array([transformed_point_mat[0][3], transformed_point_mat[1][3], transformed_point_mat[2][3]])

        return transformed_point

    def _cylinder_mesh(self, p0, p1, R):
        """
        Creates a Cylinder mesh between two points in cartesian space with a respective radius (Model for each Link of robot)

        @param nparray p0: Starting Point of Cylinder in Cartesian Space. [x, y, z]
        @param nparray p1: Ending Point of Cylinder in Cartesian Space. [x, y, z]
        @param int R: Radius of Cylinder

        Output
       @return : list [X, Y, Z] : Mesh of Cylinder (Formatted for matplotlib plot_surface function)
        """
        reso = 5

        #vector in direction of axis
        v = p1 - p0
        #find magnitude of vector
        mag = norm(v)
        #unit vector in direction of axis
        v = v / mag
        #make some vector not in the same direction as v
        not_v = np.array([1, 0, 0])
        if (v == not_v).all():
            not_v = np.array([0, 1, 0])
        #make vector perpendicular to v
        n1 = np.cross(v, not_v)
        #normalize n1
        n1 /= norm(n1)
        #make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        #surface ranges over t from 0 to length of axis and 0 to 2*pi
        t = np.linspace(0, mag, reso)
        theta = np.linspace(0, 2 * np.pi, reso)
        #use meshgrid to make 2d arrays
        t, theta = np.meshgrid(t, theta)
        #generate coordinates for surface
        X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]

        return X,Y,Z

    def _create_link_mesh(self):
        """
        Creates Mesh for Links if a Matplotlib representation is needed
        """

        robot_joint_positions = self.indy.get_DH_positions()
        robot_link_radius = self.indy.get_link_radius()
        for link in xrange(len(robot_link_radius)):
            frame_0 =  robot_joint_positions[0]
            robot_joint_positions = np.delete(robot_joint_positions, 0, 0)
            frame_1 =  robot_joint_positions[0]

            # ax.plot([joint_start[0], joint_end[0]], [joint_start[1], joint_end[1]], [joint_start[2], joint_end[2]], color = 'r') 

            X,Y,Z = self._cylinder_mesh(frame_0, frame_1, robot_link_radius['link{}'.format(link)])

            self.ax.plot_surface(X, Y, Z, color = 'b')

def parabolic_ellipsoid_mesh(p0, p1, R0, R2):
    """
    Creates a mesh in the form of a Elliptical Paraboloid between two points in cartesian space with a respective radius (Model for each Sensor of robot)

    @param nparray p0: Starting Point of Cylinder in Cartesian Space. [x, y, z]
    @param nparray p1: Ending Point of Cylinder in Cartesian Space. [x, y, z]
    @param int R0: Starting Radius in the X direction
    @param int R1: Ending Radius in the X direction
    @param int R2: Starting Radius in the Y direction
    @param int R3: Ending Radius in the Y direction

    Output
    @return : list [X, Y, Z] : Mesh (Formatted for matplotlib plot_surface function)
    """


    reso = 10

    # vector in direction of axis
    v = p1 - p0
    # find magnitude of vector
    mag = norm(v)
    # unit vector in direction of axis
    v = v / mag
    # make some vector not in the same direction as v
    not_v = np.array([1, 1, 0])
    if (v == not_v).all():
        not_v = np.array([0, 1, 0])
    # make vector perpendicular to v
    n1 = np.cross(v, not_v)
    # print n1,'\t',norm(n1)
    # normalize n1
    n1 /= norm(n1)
    # make unit vector perpendicular to v and n1
    n2 = np.cross(v, n1)
    # surface ranges over t from 0 to length of axis and 0 to 2*pi
    t = np.linspace(0, mag, reso)
    theta = np.linspace(0, 2 * np.pi, reso)

    # use meshgrid to make 2d arrays
    t, theta = np.meshgrid(t, theta)
    R_x = np.linspace(R0, 0.000001, reso)
    R_y = np.linspace(R2, 0.000001, reso)
    # generate coordinates for surface
    X, Y, Z = [p0[i] + v[i] * t + np.sqrt(R_y) *
            np.sin(theta) * n1[i] + np.sqrt(R_x) * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    
    return [X, Y, Z]

def cone_mesh(p0, p1, FOV):
    """
    Creates a mesh in the form of a Elliptical Paraboloid between two points in cartesian space with a respective radius (Model for each Sensor of robot)

    @param nparray p0: Starting Point of Cylinder in Cartesian Space. [x, y, z]
    @param nparray p1: Ending Point of Cylinder in Cartesian Space. [x, y, z]
    @param int FOV: Sensor FOV angle (degree)

    Output
    @return : list [X, Y, Z] : Mesh (Formatted for matplotlib plot_surface function)
    """
    reso1 = 7
    reso2 = 3
    # vector in direction of axis
    v = p1 - p0
    # find magnitude of vector
    mag = norm(v)
    # unit vector in direction of axis
    v = v / mag
    # make some vector not in the same direction as v
    not_v = np.array([1, 1, 0])
    if (v == not_v).all():
        not_v = np.array([0, 1, 0])
    # make vector perpendicular to v
    n1 = np.cross(v, not_v)
    # print n1,'\t',norm(n1)
    # normalize n1
    n1 /= norm(n1)
    # make unit vector perpendicular to v and n1
    n2 = np.cross(v, n1)
    # surface ranges over t from 0 to length of axis and 0 to 2*pi
    t1 = np.linspace(0, mag*np.cos(np.deg2rad(FOV/2)), reso1)
    t2 = np.linspace(mag*np.cos(np.deg2rad(FOV/2)), mag, reso2)
    R1 = np.linspace(0, mag*np.sin(np.deg2rad(FOV/2)), reso1)
    R2 = np.linspace(mag*np.sin(np.deg2rad(FOV/2)), 0, reso2)
    theta = np.linspace(0, 2 * np.pi, reso1+reso2)

    # use meshgrid to make 2d arrays
    t1, theta1 = np.meshgrid(t1, theta)
    t2, theta2 = np.meshgrid(t2, theta)
    
    # generate coordinates for surface
    X_1, Y_1, Z_1 = [p0[i] + v[i] * t1 + np.array(R1) * np.sin(theta1) * n1[i] + np.array(R1) * np.cos(theta1) * n2[i] for i in [0, 1, 2]]
    X_2, Y_2, Z_2 = [p0[i] + v[i] * t2 + np.array(R2) * np.sin(theta2) * n1[i] + np.array(R2) * np.cos(theta2) * n2[i] for i in [0, 1, 2]]

    X = np.append(X_1, X_2, axis=1)
    Y = np.append(Y_1, Y_2, axis=1)
    Z = np.append(Z_1, Z_2, axis=1)

    return [X, Y, Z]

# def cone_mesh(max_distance, FOV_angle):
#     reso = 10
#     X = np.empty((0,reso), float)
#     Y = np.empty((0,reso), float)
#     Z = np.empty((0,reso), float)
#     distance_range = np.linspace(max_distance, 0.000001, 5)
#     x_angle_range = np.linspace(np.deg2rad(-FOV_angle), np.deg2rad(FOV_angle), reso)
#     z_angle_range = np.linspace(0, 2 * np. pi, reso)

#     x_angle, z_angle = np.meshgrid(x_angle_range, z_angle_range)

#     for distance in distance_range:
#         temp_X = distance * np.sin(x_angle) * (-np.sin(z_angle))
#         temp_Y = distance * np.sin(x_angle) * np.cos(z_angle)
#         temp_Z = distance * np.cos(x_angle)
#         X = np.append(X, temp_X, axis=0)
#         Y = np.append(Y, temp_Y, axis=0)
#         Z = np.append(Z, temp_Z, axis=0)

#     return [X, Y, Z]

class sensor_bundle:
    sensor1 = cone_mesh(np.array([0.0, -0.2, 0.0]), np.array([0.0, -1.0, 0.0]) ,25)
    sensor2 = cone_mesh(np.array([0.0, 0.2, 0.0]), np.array([0.0, 1.0, 0.0]) ,25)
    sensor3 = cone_mesh(np.array([-0.2, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]) ,25)
    sensor4 = cone_mesh(np.array([0.2, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]) ,25)

    # X = np.concatenate([sensor1[0], sensor2[0], sensor3[0], sensor4[0]], axis=1)
    # Y = np.concatenate([sensor1[1], sensor2[1], sensor3[1], sensor4[1]], axis=1)
    # Z = np.concatenate([sensor1[2], sensor2[2], sensor3[2], sensor4[2]], axis=1)

    # sensor_mesh = [X, Y, Z]

    def __init__(self, DH, parent_link):
        self.DH = DH
        self.parent_link = parent_link
        self.collision_model = [self.set_collision_model(self.sensor1),
                                self.set_collision_model(self.sensor2),
                                self.set_collision_model(self.sensor3),
                                self.set_collision_model(self.sensor4)]
        self.collision_count = [0 for _ in range(4)]

    def set_collision_model(self, sensor_mesh):
        sensor_config = {
        'DH' : self.DH,
        'parent_link' : self.parent_link,
        'sensor_mesh' : sensor_mesh
        }

        collision_model = SelfRecognition(np.deg2rad([0, 0, 0, 0, 0, 0]), sensor_config, show = False)

        return collision_model

    def get_collision_score(self, data_num):
        each_score = [1.0-float(self.collision_count[i])/float(data_num) for i in range(4)]
        total_score = np.mean(each_score)

        print(each_score)
        print("total collision score : %f\n" %(1.0-total_score))

    def get_range_score(self, data_num, angle_resol, link_name):
        range_level = link_name.index(self.parent_link)




if __name__ == "__main__":
    '''
    # sensor_config = {
    # 'DH' = [alpha, theta, a ,d]
    # 'parent_link' = 'jointnum'  (Ex. 'joint0' or 'joint5')
    # 'sensor_mesh' = [X, Y, Z]
    # }

    # sensor_config = {
    # 'DH' : [-np.pi, 0, 0.0, 0.0],
    # 'parent_link' : 'joint5',
    # 'sensor_mesh' : sensor_bundle.sensor_mesh
    # }
    '''

    ## Example Implementation

    # Creating Mesh (can insert own mesh)
    # sensor_mesh = parabolic_ellipsoid_mesh(np.array([0.0, 0.0, 0.0]), np.array([0.0, -0.112, 0.0]) ,0.0025, 0.0009)
    # sensor_mesh = cone_mesh(np.array([0.0, 0.0, 0.0]), np.array([0.0, -0.8, 0.0]) ,25)

    robot_model = FK([0, 0, 0, 0, 0, 0])
    link_name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    
    bundles_index = [0 for _ in range(len(link_name))]
    for i in range(len(bundles_index)):
        link_bundle_index = [0, 0, 0]
        for j in range(3):
            # link_distance = sum(robot_model.DH_d[robot_model.link_name[link_name[i]]:robot_model.link_name[link_name[i+1]]])/4*(j+1)
            DH_d = robot_model.DH_d[robot_model.link_name[link_name[i]]+1]/4*(j+1)
            DH_a = robot_model.DH_a[robot_model.link_name[link_name[i]]+1]/4*(j+1)
            if abs(robot_model.DH_a[robot_model.link_name[link_name[i]]+1]) > abs(robot_model.DH_d[robot_model.link_name[link_name[i]]+1]):
                link_bundle_index[j] = sensor_bundle([0.0, 0.0, DH_a, 0.0], link_name[i])
            else:
                link_bundle_index[j] = sensor_bundle([0.0, 0.0, 0.0, DH_d], link_name[i])
        bundles_index[i] = link_bundle_index

    # print(robot_model.DH_alpha)
    # print(robot_model.DH_a)
    # print(robot_model.DH_d)
    # print(robot_model.link_name)

    # joint_pose = [0, 0, 0, 0, 0, 0]
    # bundles_index[1][2].collision_model[0].update_joint_states(np.deg2rad(joint_pose))
    # print(bundles_index[1][2].collision_model[0].self_collision_check())
    # print(bundles_index[2][0].collision_count)



    angle_resol = 2

    joint1 = 0
    joint2_range = np.linspace(-175, 175, angle_resol)
    joint3_range = np.linspace(-175, 175, angle_resol)
    joint4_range = np.linspace(-175, 175, angle_resol)
    joint5_range = np.linspace(-175, 175, angle_resol)
    joint6 = 0

    data_num = len(joint2_range) * len(joint3_range) * len(joint4_range) * len(joint5_range)

    count = 0
    collision_count = 0

    for joint2 in joint2_range:
        for joint3 in joint3_range:
            for joint4 in joint4_range:
                for joint5 in joint5_range:
                    joint_pose = [joint1, joint2, joint3, joint4, joint5, joint6]

                    ### check all sensor
                    for i in range(len(bundles_index)):
                        for j in range(3):
                            for k in range(4):
                                bundles_index[i][j].collision_model[k].update_joint_states(np.deg2rad(joint_pose))
                                if bundles_index[i][j].collision_model[k].self_collision_check():
                                    bundles_index[i][j].collision_count[k] += 1
                    count += 1
                    print("checking : %d / %d" %(count, data_num))


    ## print score
    for i in range(len(bundles_index)):
        for j in range(3):
            # each_score = [float(bundles_index[i][j].collision_count[k])/float(data_num) for k in range(4)]
            # total_score = np.mean(each_score)
            print("link %d, bundle %d" %(i+1, j+1))
            # print(each_score)
            # print("total score : %f\n" %(total_score))
            bundles_index[i][j].get_collision_score(data_num)


    # collision_score = float(collision_count)/float(data_num)
    # print(collision_score)

                    