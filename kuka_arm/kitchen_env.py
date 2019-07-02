import time

import pybullet as p

start_time = time.time()

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])

# Dynamic objects
beer_bottle = p.loadURDF("beer_bottle.urdf", basePosition=[3.4, 1.5, 1.1])
bowl = p.loadURDF("bowl.urdf", basePosition=[3.4, 1.5, 1.1])
plate = p.loadURDF("plate.urdf", basePosition=[3.4, 1, 1.1])
glass = p.loadURDF("glass.urdf", basePosition=[3.5, 1.3, 1.1])
knife = p.loadURDF("knife.urdf", basePosition=[3.6, 1.7, 1.1], globalScaling=0.01)
spoon = p.loadURDF("spoon.urdf", basePosition=[3.5, 1.2, 1.1], globalScaling=0.01)
fork = p.loadURDF("fork.urdf", basePosition=[3.4, 1.65, 1.1], globalScaling=1.0)
tissue_box = p.loadURDF("tissue_box.urdf", basePosition=[3.5, 0.7, 1.1], globalScaling=0.016)
tissue_box2 = p.loadURDF("tissue_box.urdf", basePosition=[3.5, 6, 1.5], globalScaling=0.016)
banana = p.loadURDF("banana.urdf", basePosition=[3.6, 2.3, 1.1], globalScaling=1.0)
crisps_chips = p.loadURDF("crisps_chips.urdf", basePosition=[3.5, 2.4, 1.1], globalScaling=0.01)
crisps_chips2 = p.loadURDF("crisps_chips.urdf", basePosition=[3.45, 6.35, 1.5], globalScaling=0.01)
teacup_model1 = p.loadURDF('TeaCup.urdf', basePosition=[3.4, 2, 1.1], baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)
teacup_model2 = p.loadURDF('TeaCup.urdf', basePosition=[3.6, 5.8, 1.55], baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)
teacup_model3 = p.loadURDF('TeaCup.urdf', basePosition=[1.5, 5.8, 1.5], baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)

# Mostly static
sofa = p.loadURDF("sofa.urdf", basePosition=[0.5, 6, 1], baseOrientation=quat_orientation)
table = p.loadURDF("table.urdf", basePosition=[1.5, 6, 1], baseOrientation=quat_orientation)
table2 = p.loadURDF("table.urdf", basePosition=[3.5, 6, 1], globalScaling=1.5,
                   baseOrientation=quat_orientation)
trash_can1 = p.loadURDF("trash_can.urdf", basePosition=[3.5, 9, 1], baseOrientation=quat_orientation)
trash_can2 = p.loadURDF("trash_can.urdf", basePosition=[3.5, 4, 1], baseOrientation=quat_orientation)
bed = p.loadURDF("bed.urdf", basePosition=[1.1, 7.8, 1], globalScaling=1.7, baseOrientation=quat_orientation)
lamp = p.loadURDF("nordic_floor_lamp_obj.urdf", basePosition=[1.5, 9, 1.3], globalScaling=0.01,
                 baseOrientation=quat_orientation)


walls = p.loadURDF("kitchen_walls.urdf")
kitchen = p.loadURDF("kitchen.urdf", basePosition=[0.2, 0.2, 0.1])

# todo attempt to make kitchen static while still keeping cupboards openable
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

print('Time taken to load all objects and begin simulation: {:.2f}'.format(time.time() - start_time))
time.sleep(4)

for i in range(10000000):
   p.stepSimulation()
   _, _, rgb, depth, seg = p.getCameraImage(320, 200, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
   # import pdb;pdb.set_trace()
   time.sleep(1/ 240)
