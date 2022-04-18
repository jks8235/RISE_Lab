
import matplotlib.pyplot as plt
from pytransform3d.urdf import UrdfTransformManager


URDF = """
<?xml version="1.0"?>
  <robot name="collision">
    <link name="collision">
      <collision name="collision_01">
        <origin xyz="0 0 1" rpy="1 0 0"/>
        <geometry>
          <sphere radius="0.2"/>
        </geometry>
      </collision>

      <collision name="collision_02">
        <origin xyz="0 0.5 0" rpy="0 1 0"/>
        <geometry>
          <cylinder radius="0.1" length="2"/>
        </geometry>
      </collision>

      <collision name="collision_03">
        <origin xyz="0 0.5 0" rpy="0 0 1"/>
        <geometry>
          <box size="0.3 0.4 0.5"/>
        </geometry>
      </collision>

      <collision name="collision_04">
        <origin xyz="-1 0 0" rpy="0 0 1"/>
        <geometry>
          <cone radius="0.1" height="1.5" />
        </geometry>
      </collision>
  </robot>
"""

tm = UrdfTransformManager()
tm.load_urdf(URDF)
ax = tm.plot_frames_in("collision", s=0.1)
tm.plot_collision_objects("collision", ax)
ax.set_zlim((-0.5, 1.5))
print(tm.check_consistency())
plt.show()