# import set
import numpy as np

# Machine set
    # Link Length set : Unit is cm
Link_1 = 1
Link_2 = 3
Link_3 = 1
Link_4 = 2

    # Joint angle set : Unit is deg
Joint_1 = 90
Joint_2 = 0
Joint_3 = 0

    # DH Parameter set

alpha_0 = 0
alpha_1 = 90
alpha_2 = 0
alpha_3 = 0

a_0 = 0
a_1 = 0
a_2 = Link_3
a_3 = Link_4

d_1 = Link_1 + Link_2
d_2 = 0
d_3 = 0
d_4 = 0

theta_1 = Joint_1
theta_2 = Joint_2
theta_3 = Joint_3
theta_4 = 0

# Transformation marix
def Transform(alpha, a, d, theta) :
    Transform_Matrix = np.zeros((4,4))

    # Angle Unit change : deg 2 rad
    alpha = alpha * np.pi / 180
    theta = theta * np.pi / 180

    # Fill in Transformation matrix
    Transform_Matrix[0,0] = np.cos(theta)
    Transform_Matrix[0,1] = -np.sin(theta)
    Transform_Matrix[0,3] = a
    Transform_Matrix[1,0] = np.sin(theta)*np.cos(alpha)
    Transform_Matrix[1,1] = np.cos(theta)*np.cos(alpha)
    Transform_Matrix[1,2] = -np.sin(alpha)
    Transform_Matrix[1,3] = -d*np.sin(alpha)
    Transform_Matrix[2,0] = np.sin(theta)*np.sin(alpha)
    Transform_Matrix[2,1] = np.cos(theta)*np.sin(alpha)
    Transform_Matrix[2,2] = np.cos(alpha)
    Transform_Matrix[2,3] = d*np.cos(alpha)
    Transform_Matrix[3,3] = 1

    return Transform_Matrix

# Transformation point to point
Trans_01 = Transform(alpha_0, a_0, d_1, theta_1)
Trans_12 = Transform(alpha_1, a_1, d_2, theta_2)
Trans_23 = Transform(alpha_2, a_2, d_3, theta_3)
Trans_34 = Transform(alpha_3, a_3, d_4, theta_4)

Trans_02 = np.dot(Trans_01, Trans_12)
Trans_13 = np.dot(Trans_12, Trans_23)
Trans_24 = np.dot(Trans_23, Trans_34)

Trans_03 = np.dot(Trans_02, Trans_23)
Trans_14 = np.dot(Trans_13, Trans_34)

Trans_04 = np.dot(Trans_03, Trans_34)

# result output
print(Trans_04)