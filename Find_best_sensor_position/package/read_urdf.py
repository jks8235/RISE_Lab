#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import os
import matplotlib.pyplot as plt
from pytransform3d.urdf import UrdfTransformManager
import math

data_dir = "/home/jee/work_space/catkin_wk/src/RISE_Lab/Find_best_sensor_position/package/puma560/urdf/puma560_robot.urdf"


tm = UrdfTransformManager()
with open(data_dir, "r") as f:
    tm.load_urdf(f.read(), mesh_path=data_dir)

joint_names = ["j%d" % i for i in range(1,6+1)]
joint_angles = [90, 60, 0, 90, 0, 0]
for name, angle in zip(joint_names, joint_angles):
    tm.set_joint(name, math.radians(angle))

tm.set_joint("j1", math.radians(0))
tm.set_joint("j2", math.radians(0))
tm.set_joint("j3", math.radians(0))
tm.set_joint("j4", math.radians(0))
tm.set_joint("j5", math.radians(0))
tm.set_joint("j6", math.radians(0))

ax = tm.plot_frames_in(
    "Puma560", s=0.1, whitelist=["link%d" % d for d in range(1, 6+1)], show_name=True)
ax = tm.plot_connections_in("Puma560", ax=ax)
tm.plot_visuals("Puma560", ax=ax)
tm.plot_connections_in("Puma560", ax=ax)
# tm.plot_collision_objects("Puma560", ax)
ax.set_xlim((-1.0, 1.0))
ax.set_ylim((-1.0, 1.0))
ax.set_zlim((-0.1, 2.0))
plt.show()

print(tm.check_consistency())