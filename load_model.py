import time

import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")  # todo why gap between ground at start? Probably to do with origin, scale and collision properties

vector_start_position = [0, 0, 10]
vector_model = p.loadSDF('vector_model.sdf', globalScaling=10.0)

# <scale>0.0280866 0.0281665 0.0197164/scale>
# <scale>.01 .01 .01</scale>
teacup_model = p.loadSDF('teacup_model.sdf')
# teacup_model = p.loadSDF('teacup_model_mesh_collision.sdf')

print(vector_model)

for i in range(10000):
   p.stepSimulation()
   time.sleep(1/ 240)
   #p.changeDynamics(vector_model[0], -1)

time.sleep(20)