from scipy.spatial.transform import Rotation as R
from sympy import degree

zyz = R.as_matrix(R.from_euler('zyz', [0, 90, 0], degrees=True))
print(zyz)