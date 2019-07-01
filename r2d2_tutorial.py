import random
import time

import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1])



for i in range(10000):
   p.stepSimulation()
   time.sleep(1/ 240)
   # time.sleep(1)
   # p.changeDynamics(vector_model[0], -1)

# time.sleep(20)
