import pybullet as p
import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
	
p.resetSimulation()
p.setGravity(0,0,-10)
useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim) # either this
# p.loadURDF("plane.urdf")
p.loadSDF("stadium.sdf")

pixelWidth = 320
pixelHeight = 220
camTargetPos = [0, 0, 0]
camDistance = 4
camDistance = 0.5
pitch = -10.0
roll = 0
upAxisIndex = 2

# car = p.loadURDF("data/racecar_differential.urdf") #, [0,0,2],useFixedBase=True)
# for i in range (p.getNumJoints(car)):
# 	print (p.getJointInfo(car,i))
# for wheel in range(p.getNumJoints(car)):
# 		p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
# 		p.getJointInfo(car,wheel)
#
# wheels = [8,15]
# print("----------------")
#
# #p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
# c = p.createConstraint(car,9,car,11,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=1, maxForce=10000)
#
# c = p.createConstraint(car,10,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=-1, maxForce=10000)
#
# c = p.createConstraint(car,9,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=-1, maxForce=10000)
#
# c = p.createConstraint(car,16,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=1, maxForce=10000)
#
#
# c = p.createConstraint(car,16,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=-1, maxForce=10000)
#
# c = p.createConstraint(car,17,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=-1, maxForce=10000)
#
# c = p.createConstraint(car,1,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15, maxForce=10000)
# c = p.createConstraint(car,3,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
# p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15,maxForce=10000)
#
#
# steering = [0,2]
#
# targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
# maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
# steeringSlider = p.addUserDebugParameter("steering",-1,1,0)
while (True):
	# maxForce = p.readUserDebugParameter(maxForceSlider)
	# targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
	# steeringAngle = p.readUserDebugParameter(steeringSlider)
	# #print(targetVelocity)
	#
	# for wheel in wheels:
	# 	p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
    #
	# for steer in steering:
	# 	p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=-steeringAngle)
		
	# steering

	for yaw in range(0, 360, 10):

		viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll,
														 upAxisIndex)
		projectionMatrix = [1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0, 0.0,
							0.0, -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0]

		start = time.time()
		img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix=viewMatrix,
								   projectionMatrix=projectionMatrix, shadow=1,
								   lightDirection=[1, 1, 1])

		if (useRealTimeSim==0):
			p.stepSimulation()
		time.sleep(0.01)

	camTargetPos = [camTargetPos[0] + 0.1, camTargetPos[2], camTargetPos[2]]