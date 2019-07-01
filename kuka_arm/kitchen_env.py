import random
import time

import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# walls = p.loadURDF("kitchen_walls.urdf")
quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])
# sofa = p.loadURDF("sofa.urdf", basePosition=[0.5, 7, 1], baseOrientation=quat_orientation)
# table = p.loadURDF("table.urdf", basePosition=[1.5, 7, 1], baseOrientation=quat_orientation)
# table = p.loadURDF("trashcan.urdf", basePosition=[3, 8.5, 1], baseOrientation=quat_orientation)
# bed = p.loadURDF("bed.urdf", basePosition=[0.55, 9, 1], baseOrientation=quat_orientation)
# bed = p.loadURDF("nordic_floor_lamp_obj.urdf", basePosition=[1.5, 9, 0.0], globalScaling=0.01,
bed = p.loadURDF("nordic_floor_lamp_obj.urdf", basePosition=[0.5, 0, 1.0], globalScaling=0.01,
                 baseOrientation=quat_orientation)


# kitchen = p.loadURDF("kitchen.urdf", basePosition=[0.2, 0.2, 0])
# kitchen = p.loadURDF("kitchen.urdf")
# # p.loadURDF("IAI_kitchen.urdf", basePosition=[0, 0, 0.2])
# numJoints = p.getNumJoints(kitchen)
# for jointIndex in range(numJoints):
#     #p.resetJointState(kitchen, jointIndex, jointPositions[jointIndex])
#     print(p.getJointInfo(kitchen, jointIndex))
#     print(p.getDynamicsInfo(kitchen, jointIndex))
#     # p.changeDynamics(kitchen, jointIndex, mass=1000000)
#     # print(p.getDynamicsInfo(kitchen, jointIndex))
#
# pos = [3.3, 2, 1]
# quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])
# teacup_model = p.loadURDF('TeaCup.urdf', basePosition=pos, baseOrientation=quat_orientation,
#                           flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)


# time.sleep(2)

for i in range(10000):
   p.stepSimulation()
   time.sleep(1/ 240)
