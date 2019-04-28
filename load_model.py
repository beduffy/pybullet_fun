import time

import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")  # todo why gap between ground at start? Probably to do with origin, scale and collision properties

vector_start_position = [0, 0, 2]
# vector_model = p.loadSDF('model.sdf', globalScaling=10.0)
vector_model = p.loadURDF('vector_from_sdf.urdf', globalScaling=5.0, basePosition=vector_start_position,
# vector_model = p.loadURDF('vector.urdf', globalScaling=5.0,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL
                                                       )




# <scale>0.0280866 0.0281665 0.0197164/scale>
# <scale>.01 .01 .01</scale>
pos = [0, 0, 2]
quat_orientation = p.getQuaternionFromEuler([1, 0, 0])
# teacup_model = p.loadSDF('teacup_model.sdf')
# teacup_model = p.loadSDF('teacup_model_mesh_collision.sdf')
# teacup_model = p.loadURDF('TeaCup.urdf', basePosition=pos, baseOrientation=quat_orientation, flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
# teacup_model = p.loadURDF('coffee_cup.urdf', basePosition=pos,
#                           baseOrientation=quat_orientation, flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
#                           globalScaling=0.1)
#
# print(vector_model)

time.sleep(2)

for i in range(10000):
   p.stepSimulation()
   time.sleep(1/ 240)
   # time.sleep(1)
   # p.changeDynamics(vector_model[0], -1)

# time.sleep(20)
