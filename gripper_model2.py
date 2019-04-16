import pybullet as p
import time

p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

# sphereRadius = 0.05
# sphereRadius = 0.5
# colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)

boxHalfLength = 0.05
boxHalfWidth = 1.0
boxHalfHeight = 0.07
# segmentLength = 5

colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])

link_Masses = [1]
linkCollisionShapeIndices = [colBoxId]
linkVisualShapeIndices = [-1]
linkPositions = [[0, 0.5, 0]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[1, 0, 0]]
baseOrientation = [1, 0, 0, 1]

boxId = p.createMultiBody(0, -1, -1, [0, 0, 1.5], baseOrientation,
                              linkMasses=link_Masses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions, linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=indices, linkJointTypes=jointTypes,
                              linkJointAxis=axis)
# p.changeDynamics(boxId, -1, spinningFriction=0.001, rollingFriction=0.001, linearDamping=0.0)

p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=5,
                                        force=10)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

# p.getNumJoints(sphereUid)
# for i in range(p.getNumJoints(sphereUid)):
#     p.getJointInfo(sphereUid, i)

while (1):
    keys = p.getKeyboardEvents()
    # print(keys)

    # time.sleep(0.01)
