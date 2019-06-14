import random
import time

import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")
p.loadURDF("kitchen.urdf")
# p.loadURDF("IAI_kitchen.urdf", basePosition=[0, 0, 0.2])


pos = [3.3, 2, 1]
quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])
teacup_model = p.loadURDF('TeaCup.urdf', basePosition=pos, baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)


time.sleep(2)

for i in range(10000):
   p.stepSimulation()
   time.sleep(1/ 240)
