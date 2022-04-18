import numpy as np
from math import sin as s
from math import cos as c
import copy
from scipy.spatial.transform import Rotation as Rot

#### TO DO ####
# Go through other classes and remove get.... commands and call for self properties instead
# Then remove unnecessary get commands

# Create _compute_joint_jacobians and get_joint_jacobian(joint)


def t_mat(DH_alpha, DH_theta, DH_a, DH_d):
    ct = c(DH_theta)
    st = s(DH_theta)
    ca = c(DH_alpha)
    sa = s(DH_alpha)
    return np.array([
        [ct,        -st,        0.0,    DH_a],
        [st * ca,   ct * ca,    -sa,    -DH_d * sa],
        [st * sa,   ct * sa,    ca,     DH_d * ca],
        [0.0,       0.0,        0.0,    1.0],
    ])


class FKIndy7:
    link_name = {'joint0':0,'joint1':2,'joint2':3,'joint3':5,'joint4':7,'joint5':9}
    link_radius = {'link0': 0.07, 'link1': 0.07, 'link2': 0.07, 'link3': 0.06, 'link4': 0.055, 'link5': 0.05, 'link6': 0.05}
    link_DH = [True, True, True, True, True, True, False, True, False, True, True]
    real_joint = [True, False, True, True, False,
                  True, False, True, False, True, False]
    joint_limits = np.array(np.deg2rad([[-175, -175, -175, -175, -175, -215],
                                        [175, 175, 175, 175, 175, 215]]))

    DH_alpha = [0.0, 0.0, np.pi / 2, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0]
    DH_a = [0.0, 0.0, 0.0, -0.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    DH_d = [0.0770, 
    , 0.109, -0.0305, -0.075, 0.267, 0.083, 0.114, 0.069, 0.168, .06]

    def __init__(self, joint_angles):
        self.num_joints = len(joint_angles)
        self.update_joints(joint_angles)

    def get_jacobian_matrix(self):
        """
        Return Jacobian Matrix
        """
        return self._jacobian_matrix

    def get_joint_jacobians(self):
        joint_jacobians = [self._compute_jacobian_at_joint(joint) for joint in ['joint0', 'joint1', 'joint2','joint3', 'joint4', 'joint5']]
       
        return joint_jacobians


    def get_ee_pos(self):
        """
        Return End Effector Position
        """
        return self.ee_pos

    def get_ee_ori(self, method='quat'):
        """
        Return Orientation

        Input:
        :param 'string' method: Representation of orientation : 'matrix', 'quat', 'rotvec', 'euler'
        """
        r = Rot.from_dcm(list(self.t_global[:3, :3]))

        if method is 'matrix':
            ori = r.as_dcm()
        elif method is 'quat':
            ori = r.as_quat()
        elif method is 'rotvec':
            ori = r.as_rotvec()
        elif method is 'euler':
            ori = r.as_euler('xyz', degrees=False)
        else:
            raise NotImplementedError

        return ori

    def get_joint_positions(self):
        return self.joint_positions

    def get_joint_trans_list(self):
        return self.real_joint_trans_list

    def get_DH_positions(self):
        return self.DH_positions

    def get_link_radius(self):
        return self.link_radius

    def get_tran_matrix(self):
        return self.t_global

    def get_tran_matrix_at_joint(self, joint):

        tran_matrix = self._compute_tran_to(self.link_name[joint])

        return tran_matrix
            
    def print_trans_between_joints(self):
        prev_joint = 0
        for joint in xrange(len(self.real_joint)):

            if self.real_joint[joint]:
                print(self._get_trans_from(0, joint))

                prev_joint = joint

    def get_num_joints(self):
        """
        Return number of Real Joints
        """
        return self.num_joints

    def get_t_list(self):
        for i in xrange(len(self.t_list)):
            print(self.t_list[i])

    def get_joint_limits(self):
        return self.joint_limits

    def update_joints(self, target_joint_angles):
        """
        Update Kinematics of Robot using new Joint Positions (rad)
        """

        if len(target_joint_angles) != self.num_joints:
            raise ValueError

        self.joint_angles = [
            target_joint_angles[0],
            0.0,
            target_joint_angles[1] - np.pi / 2,
            target_joint_angles[2],
            np.pi / 2,
            target_joint_angles[3],
            0.0,
            target_joint_angles[4] - np.pi / 2,
            np.pi / 2,
            target_joint_angles[5],
            0.0,
        ]

        self._update()

    def _update(self):
        """
        Compute New Transformations, End Effector Positions, and Jacobian
        """

        # DH Parameters
        DH_alpha = [0.0, 0.0, np.pi / 2, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0]
        DH_a = [0.0, 0.0, 0.0, -0.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        DH_d = [0.0770, 0.222, 0.109, -0.0305, -0.075, 0.267, 0.083, 0.114, 0.069, 0.168, .06]

        # List of Tranformation Matrices
        self.t_list = [
            t_mat(_alpha, _theta, _a, _d)
            for _alpha, _theta, _a, _d in zip(DH_alpha, self.joint_angles, DH_a, DH_d)
        ]

        # Transformation Matrix from Frame 0 to EE
        self.t_global = t_mat(0.0, 0.0, 0.0, 0.0)
        for T in self.t_list:
            self.t_global = np.dot(self.t_global, T)

        # Update Parameters
        self.ee_pos = self.t_global[:3, 3]  # End effector Position
        self._compute_real_joint_trans_list()
        self._compute_real_joint_positions()  # Position of Joints wrt Base Frame
        self._compute_jacobian()  # Jacobian Matrix
        self._compute_all_DH_positions() # Positon of All DH Frames

    def _compute_jacobian(self):
        """
        Compute Jacobian Matrix
        """

        # Initializing Jacobian Arrays for Position and Orientation
        jacobian_matrix_pos = np.array([[], [], []])
        jacobian_matrix_ori = np.array([[], [], []])

        # Index Counter
        index_of_each_transformation_matrix = 0

        # Jacobian Calculation for each Joint
        for is_real_joint in self.real_joint:
            if is_real_joint:
                # Transformation Matrix 0 -> i
                this_tran = self._compute_tran_to(
                    index_of_each_transformation_matrix)
                # print(this_tran)
                this_rot = this_tran[:3, :3]

                # Distance Vector d_ee - d_i
                this_dist = self.ee_pos - this_tran[:3, 3]

                # J_pos = R_0toi * [0 0 1] x (d_ee - d_i)
                this_jacobian_pos = np.cross(this_rot[:, 2], this_dist)

                # J_ori = R_0toi * [0 0 1]
                # Todo : optimization
                # geo to anal
                this_jacobian_ori = this_rot[:, 2]

                # Convert to a 2D matrix
                this_jacobian_pos = np.array(
                    [[pos_vec] for pos_vec in this_jacobian_pos])
                this_jacobian_ori = np.array(
                    [[ori_vec] for ori_vec in this_jacobian_ori])

                # If real joint, add to jacobian matrix
                jacobian_matrix_pos = np.concatenate(
                    (jacobian_matrix_pos, this_jacobian_pos), axis=1
                )
                jacobian_matrix_ori = np.concatenate(
                    (jacobian_matrix_ori, this_jacobian_ori), axis=1
                )
            index_of_each_transformation_matrix += 1

        # Combine Jv and Jw
        self._jacobian_matrix = np.concatenate(
            (jacobian_matrix_pos, jacobian_matrix_ori), axis=0
        )

    def _compute_jacobian_at_joint(self, joint):
        """
        Compute Jacobian Matrix at specified joint
        """

        # Initializing Jacobian Arrays for Position and Orientation
        jacobian_matrix_pos = np.array([[], [], []])
        jacobian_matrix_ori = np.array([[], [], []])

        # Index Counter
        index_of_each_transformation_matrix = 0

        # Jacobian Calculation for each Joint
        last_joint = self.link_name[joint]
        # print(last_joint)
        for is_real_joint in self.real_joint:
            if is_real_joint:
                # Transformation Matrix 0 -> i
                this_tran = self._compute_tran_to(
                    index_of_each_transformation_matrix)
                # print(this_tran)
                this_rot = this_tran[:3, :3]

                # Distance Vector d_ee - d_i
                this_dist = self.ee_pos - this_tran[:3, 3]

                # J_pos = R_0toi * [0 0 1] x (d_ee - d_i)
                this_jacobian_pos = np.cross(this_rot[:, 2], this_dist)

                # J_ori = R_0toi * [0 0 1]
                # Todo : optimization
                # geo to anal
                this_jacobian_ori = this_rot[:, 2]

                # Convert to a 2D matrix
                this_jacobian_pos = np.array(
                    [[pos_vec] for pos_vec in this_jacobian_pos])
                this_jacobian_ori = np.array(
                    [[ori_vec] for ori_vec in this_jacobian_ori])

                if index_of_each_transformation_matrix > last_joint:
                    # print('over')
                    this_jacobian_pos = np.array(
                        [[0] for pos_vec in this_jacobian_pos])
                    this_jacobian_ori = np.array(
                        [[0] for ori_vec in this_jacobian_ori])

                # If real joint, add to jacobian matrix
                jacobian_matrix_pos = np.concatenate(
                    (jacobian_matrix_pos, this_jacobian_pos), axis=1
                )
                jacobian_matrix_ori = np.concatenate(
                    (jacobian_matrix_ori, this_jacobian_ori), axis=1
                )
            index_of_each_transformation_matrix += 1

        # Combine Jv and Jw
        jacobian_matrix = np.concatenate(
            (jacobian_matrix_pos, jacobian_matrix_ori), axis=0
        )
        return jacobian_matrix

    def _compute_tran_to(self, end_frame):
        """
        Compute Transformation Matrix from Frame 0 to desired frame

        input:
        :param int end_frame: Desired Frame

        return: np.array Tran: Transformation Matrix from 0 -> Desired Frame
        """

        T = self.t_list[0:end_frame+1]

        Tran = reduce(np.matmul, T)

        return Tran

    def _get_trans_from(self, parent_frame, child_frame):
        """Create Transformation Matrix from Frame A to B
        Input
        :param integer index1: index of Frame A
        :param integer index2: index of Frame B

        Output
        :return array Tran: Transformation matrix from A to B
        """
        if parent_frame == child_frame:
            return self.t_list[parent_frame]

        for i in xrange(parent_frame, child_frame+1):
            if i is parent_frame:
                Tran = self.t_list[i]
            else:
                Tran = np.matmul(Tran, self.t_list[i])

        return Tran

    def _compute_real_joint_positions(self):
        """Calculate positions of each joint wrt base frame"""

        first_iter = True
        for joint in xrange(len(self.real_joint)):

            if self.real_joint[joint]:
                this_joint_trans = self._get_trans_from(0, joint)[:3, 3].T
                if first_iter == True:
                    self.joint_positions = np.array([this_joint_trans])
                    first_iter = False
                else:
                    self.joint_positions = np.append(
                        self.joint_positions, [this_joint_trans], axis=0)

    def _compute_real_joint_trans_list(self):
        """Calculate transformation matrix of each joint wrt base frame"""

        first_iter = True
        for joint in xrange(len(self.real_joint)):

            if self.real_joint[joint]:
                this_joint_trans = self._get_trans_from(0, joint)
                if first_iter == True:
                    self.real_joint_trans_list = np.array([this_joint_trans])
                    first_iter = False
                else:
                    self.real_joint_trans_list = np.append(
                        self.real_joint_trans_list, [this_joint_trans], axis=0)

    def _compute_all_DH_positions(self):
        """Calculate positions of each DH parameter wrt base frame"""

        first_iter = True
        for DH in xrange(len(self.link_DH)):
            if self.link_DH[DH]:
                this_DH_trans = self._get_trans_from(0, DH)[:3, 3].T
                if first_iter == True:
                    self.DH_positions = np.array([this_DH_trans])
                    first_iter = False
                else:
                    self.DH_positions = np.append(
                        self.DH_positions, [this_DH_trans], axis=0)

    # CURRENTLY UNUSED  ########### MEANT FOR JACOBIAN GEOMETRIC -> ANALYTICAL
    def _matrix2euler_zyx(self, mat):
        """
        Convert dcm to euler %%%%%scipy rotation can replace
        input:
        :param: nparray mat: 
        """
        rot = mat[0:3, 0:3]
        if abs(rot[0, 0]) < 1e-3 and abs(rot[1, 0]) < 1e-3 and abs(rot[2, 0]+1) < 1e-3:
            rx = np.arctan2(rot[0, 1], rot[1, 1])
            ry = np.pi/2
            rz = 0
        elif abs(rot[0, 0]) < 1e-3 and abs(rot[1, 0]) < 1e-3 and abs(rot[2, 0]-1) < 1e-3:
            rx = np.arctan2(-rot[0, 1], rot[1, 1])
            ry = -np.pi/2
            rz = 0
        else:
            rx = np.arctan2(rot[2, 1], rot[2, 2])
            ry = np.arctan2(-rot[2, 0], np.sqrt(rot[0, 0]**2+rot[1, 0]**2))
            rz = np.arctan2(rot[1, 0], rot[0, 0])
        angle = np.array([rx, ry, rz])

        return angle

    def _omega2euler_zyx(self, eul):
        rx = eul[0]
        ry = eul[1]
        rz = eul[2]
        c_inv = np.array([[np.cos(rz)/np.cos(ry),           np.sin(rz)/np.cos(ry), 0],
                          [-np.sin(rz),                   np.cos(rz), 0],
                          [(np.cos(rz)*np.sin(ry))/np.cos(ry), (np.sin(ry)*np.sin(rz))/np.cos(ry), 1]])
        return c_inv

    def _geometrical2analytical(self, J, eul):
        jaco = copy.deepcopy(J)
        E = self._omega2euler_zyx(eul)
        jaco[3:, :] = np.dot(E, jaco[3:, :])
        return jaco


if __name__ == "__main__":

    # theta = np.deg2rad([-25, -30, -30, 0, -120, 0])
    theta = np.deg2rad([0, 0, 0, 0, 0, 0])

    indy = FKIndy7(theta)

    # indy.get_transbetweenjoints()

    # print(indy.get_tran_matrix())
    # print(indy.get_ee_pos())
    # indy.get_t_list()
    
    print(indy.get_joint_jacobians())
    # print(indy.get_ee_ori())
