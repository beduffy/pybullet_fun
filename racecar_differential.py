import math
import time

import numpy as np
import pybullet as p

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)
useRealTimeSim = 1

# for video recording (works best on Mac and Linux, not well on Windows)
# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
# p.loadURDF("plane.urdf")
p.loadSDF("stadium.sdf")

car = p.loadURDF("data/racecar_differential.urdf")  # , [0,0,2],useFixedBase=True)
for i in range(p.getNumJoints(car)):
    print(p.getJointInfo(car, i))
for wheel in range(p.getNumJoints(car)):
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, wheel)

p.resetBasePositionAndOrientation(car, [0, 0, 0],[0,0,0,1])

pixelWidth = 320
pixelHeight = 220
camDistance = 4
camDistance = 0.5
upAxisIndex = 2
camera_height = 0.3

wheels = [8, 15]
print("----------------")

# p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 50, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 50, 20)
steeringSlider = p.addUserDebugParameter("steering", -1, 1, 0)
t = 0
value_to_add = 0.0
while (True):
    maxForce = p.readUserDebugParameter(maxForceSlider)
    targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
    steeringAngle = p.readUserDebugParameter(steeringSlider)
    # print(targetVelocity)

    for wheel in wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity,
                                force=maxForce)

    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=-steeringAngle)

    # if t % 200 == 0:
    car_orientation = p.getBasePositionAndOrientation(car)
    x_y_z = list(car_orientation[0])
    x_y_z[2] += camera_height
    quaternion_pose = car_orientation[1]
    # print()
    cameraUpVector = [0, 0, 1]
    # target_position = [0, 0, 0]
    euler_angle_pose = p.getEulerFromQuaternion(quaternion_pose)
    multiplier = 4.0
    multiplier = 40.0
    # yaw = math.degrees(euler_angle_pose[0]) - 90
    # pitch = math.degrees(euler_angle_pose[1])
    # roll = math.degrees(euler_angle_pose[2])
    # target_position = [x_y_z[0], x_y_z[1], x_y_z[2]]# - camera_height]
    # target_position = [x_y_z[0] + multiplier * euler_angle_pose[0],
    # 				   x_y_z[1] + multiplier * euler_angle_pose[1],
    # 				   x_y_z[2] + multiplier * euler_angle_pose[2]]# - camera_height]
    # yaw = math.degrees(euler_angle_pose[0]) - 90
    # pitch = math.degrees(euler_angle_pose[1])
    # roll = math.degrees(euler_angle_pose[2])
    # yaw = euler_angle_pose[0]  + value_to_add # - 90
    # pitch = euler_angle_pose[1]
    # roll = euler_angle_pose[2]
    pitch = euler_angle_pose[0]
    roll = euler_angle_pose[1]
    yaw = euler_angle_pose[2]
    # front_vector = [math.cos(pitch) * math.cos(yaw),
                    # math.sin(pitch),
                    # math.cos(pitch) * math.sin(yaw)]

    front_vector = [math.cos(yaw) * math.cos(pitch),
                    math.sin(yaw),
                    math.cos(yaw) * math.sin(pitch)]

    front_vector = (np.array(front_vector) / np.linalg.norm(front_vector)).tolist()

    target_position = [x_y_z[0] + multiplier * front_vector[0],
                       x_y_z[1] + multiplier * front_vector[1],
                       x_y_z[2] + multiplier * front_vector[2]]
    # target_position = [- x_y_z[0] + front_vector[0],
    #                    - x_y_z[1] + front_vector[1],
    #                    - x_y_z[2] + front_vector[2]]

    viewMatrix = p.computeViewMatrix(x_y_z, target_position, cameraUpVector)

    if t % 50 == 0:
        print('quaternion: {}'.format(quaternion_pose))
        print('yaw: {} pitch: {} roll: {}: '.format(yaw, pitch, roll))
        print('front: ', front_vector)
        print('target_position: ', target_position)
        print('x_y_z: ', x_y_z)

    # yaw = euler_angle_pose[0] - 90
    # pitch = euler_angle_pose[1]
    # roll = euler_angle_pose[2]
    # yaw = math.degrees(euler_angle_pose[0]) - 90
    # pitch = math.degrees(euler_angle_pose[1])
    # roll = math.degrees(euler_angle_pose[2])
    # viewMatrix = p.computeViewMatrixFromYawPitchRoll(x_y_z, camDistance, yaw, pitch,
    # 																			 roll,
    # 																			 upAxisIndex)

    projectionMatrix = [1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0,
                        0.0,
                        0.0, -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0]

    img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix=viewMatrix,
                               projectionMatrix=projectionMatrix, shadow=1,
                               lightDirection=[1, 1, 1])

    # viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch,
    # 												 roll,
    # 												 upAxisIndex)
    # projectionMatrix = [1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0,
    # 					0.0,
    # 					0.0, -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0]
    #
    # start = time.time()
    # img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix=viewMatrix,
    # 						   projectionMatrix=projectionMatrix, shadow=1,
    # 						   lightDirection=[1, 1, 1])

    #todo PyBullet.isNumpyEnabled(

    keys = p.getKeyboardEvents()
    if keys.get(100):
        value_to_add += 0.1
        print(value_to_add)
    # steering
    if (useRealTimeSim == 0):
        p.stepSimulation()
    time.sleep(0.01)

    t += 1
