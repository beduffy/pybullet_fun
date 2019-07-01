import random
import time

import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])

walls = p.loadURDF("kitchen_walls.urdf")
sofa = p.loadURDF("sofa.urdf", basePosition=[0.5, 6, 1], baseOrientation=quat_orientation)
table = p.loadURDF("table.urdf", basePosition=[1.5, 6, 1], baseOrientation=quat_orientation)
table2 = p.loadURDF("table.urdf", basePosition=[3.5, 6, 1], globalScaling=1.5,
                   baseOrientation=quat_orientation)
trash_can1 = p.loadURDF("trash_can.urdf", basePosition=[3.5, 9, 1], baseOrientation=quat_orientation)
trash_can2 = p.loadURDF("trash_can.urdf", basePosition=[3.5, 4, 1], baseOrientation=quat_orientation)
bed = p.loadURDF("bed.urdf", basePosition=[1.1, 7.8, 1], globalScaling=1.7, baseOrientation=quat_orientation)
lamp = p.loadURDF("nordic_floor_lamp_obj.urdf", basePosition=[1.5, 9, 1.3], globalScaling=0.01,
                 baseOrientation=quat_orientation)




kitchen = p.loadURDF("kitchen.urdf", basePosition=[0.2, 0.2, 0.1])

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
pos = [3.4, 2, 1.1]
teacup_model = p.loadURDF('TeaCup.urdf', basePosition=pos, baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)


for i in range(10000000):
   p.stepSimulation()
   # _, _, rgb, depth, seg = p.getCameraImage(320, 200, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
   # import pdb;pdb.set_trace()
   # time.sleep(1/ 240)
