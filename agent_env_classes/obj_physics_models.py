import time
import os
import numpy as np
import copy
import math

import pybullet as p

class Racecar:
    def __init__(self, bullet_client, urdfRootPath='', timeStep=0.01):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self._p = bullet_client
        self.reset()

    def reset(self):
        car = self._p.loadURDF(os.path.join(self.urdfRootPath, "racecar/racecar_differential.urdf"),
                               [0, 0, .2], useFixedBase=False)
        self.racecarUniqueId = car
        # for i in range (self._p.getNumJoints(car)):
        #	print (self._p.getJointInfo(car,i))
        for wheel in range(self._p.getNumJoints(car)):
            self._p.setJointMotorControl2(car, wheel, self._p.VELOCITY_CONTROL, targetVelocity=0,
                                          force=0)
            self._p.getJointInfo(car, wheel)

        # self._p.setJointMotorControl2(car,10,self._p.VELOCITY_CONTROL,targetVelocity=1,force=10)
        c = self._p.createConstraint(car, 9, car, 11, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=1, maxForce=10000)

        c = self._p.createConstraint(car, 10, car, 13, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

        c = self._p.createConstraint(car, 9, car, 13, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

        c = self._p.createConstraint(car, 16, car, 18, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=1, maxForce=10000)

        c = self._p.createConstraint(car, 16, car, 19, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

        c = self._p.createConstraint(car, 17, car, 19, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

        c = self._p.createConstraint(car, 1, car, 18, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
        c = self._p.createConstraint(car, 3, car, 19, jointType=self._p.JOINT_GEAR,
                                     jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                                     childFramePosition=[0, 0, 0])
        self._p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

        self.steeringLinks = [0, 2]
        self.maxForce = 20
        self.nMotors = 2
        self.motorizedwheels = [8, 15]
        self.speedMultiplier = 20.
        self.steeringMultiplier = 0.5

    def getActionDimension(self):
        return self.nMotors

    def getObservationDimension(self):
        return len(self.getObservation())

    def getObservation(self):
        observation = []
        pos, orn = self._p.getBasePositionAndOrientation(self.racecarUniqueId)

        observation.extend(list(pos))
        observation.extend(list(orn))

        return observation

    def applyAction(self, motorCommands):
        targetVelocity = motorCommands[0] * self.speedMultiplier
        # print("targetVelocity")
        # print(targetVelocity)
        steeringAngle = motorCommands[1] * self.steeringMultiplier
        # print("steeringAngle")
        # print(steeringAngle)
        # print("maxForce")
        # print(self.maxForce)

        for motor in self.motorizedwheels:
            self._p.setJointMotorControl2(self.racecarUniqueId, motor, self._p.VELOCITY_CONTROL,
                                          targetVelocity=targetVelocity, force=self.maxForce)
        for steer in self.steeringLinks:
            self._p.setJointMotorControl2(self.racecarUniqueId, steer, self._p.POSITION_CONTROL,
                                          targetPosition=steeringAngle)

class Vector:
    def __init__(self, bullet_client, urdfRootPath='', timeStep=0.01):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep

        self.discrete_actions = True
        self._p = bullet_client
        self.reset()

    def reset(self):
        # car = self._p.loadURDF(os.path.join(self.urdfRootPath, "racecar/racecar_differential.urdf"),
        #                        [0, 0, .2], useFixedBase=False)
        # car = self._p.loadURDF(os.path.join(self.urdfRootPath, "racecar/racecar_differential.urdf"),
        #                        [0, 0, .2], useFixedBase=False)
        car = p.loadSDF("model.sdf", globalScaling=3.0)  # , [0,0,2],useFixedBase=True)
        print(car)
        car = car[0]
        print(p.getNumJoints(car))
        self.racecarUniqueId = car
        # for i in range (self._p.getNumJoints(car)):
        #	print (self._p.getJointInfo(car,i))
        for wheel in range(self._p.getNumJoints(car)):
            self._p.setJointMotorControl2(car, wheel, self._p.VELOCITY_CONTROL, targetVelocity=0,
                                          force=0)
            self._p.getJointInfo(car, wheel)

        # self._p.setJointMotorControl2(car,10,self._p.VELOCITY_CONTROL,targetVelocity=1,force=10)
        # c = self._p.createConstraint(car, 9, car, 11, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=1, maxForce=10000)
        #
        # c = self._p.createConstraint(car, 10, car, 13, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)
        #
        # c = self._p.createConstraint(car, 9, car, 13, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)
        #
        # c = self._p.createConstraint(car, 16, car, 18, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=1, maxForce=10000)
        #
        # c = self._p.createConstraint(car, 16, car, 19, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)
        #
        # c = self._p.createConstraint(car, 17, car, 19, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)
        #
        # c = self._p.createConstraint(car, 1, car, 18, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
        # c = self._p.createConstraint(car, 3, car, 19, jointType=self._p.JOINT_GEAR,
        #                              jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
        #                              childFramePosition=[0, 0, 0])
        # self._p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

        self.steeringLinks = [0, 2]
        self.maxForce = 20
        self.nMotors = 2
        self.motorizedwheels = [8, 15]
        self.speedMultiplier = 20.
        self.steeringMultiplier = 0.5  # todo below into applyAction

    def getActionDimension(self):
        return self.nMotors

    def getObservationDimension(self):
        return len(self.getObservation())

    def getObservation(self):
        observation = []
        pos, orn = self._p.getBasePositionAndOrientation(self.racecarUniqueId)

        observation.extend(list(pos))
        observation.extend(list(orn))

        return observation

    def applyAction(self, motorCommands):
        targetVelocity = 1 * self.speedMultiplier  # todo go forward and backwards
        # targetVelocity = motorCommands[0] * self.speedMultiplier
        # print("targetVelocity")
        # print(targetVelocity)
        # steeringAngle = motorCommands[1] * self.steeringMultiplier
        # print("steeringAngle")
        # print(steeringAngle)
        # print("maxForce")
        # print(self.maxForce)

        # for motor in self.motorizedwheels:
        #     self._p.setJointMotorControl2(self.racecarUniqueId, motor, self._p.VELOCITY_CONTROL,
        #                                   targetVelocity=targetVelocity, force=self.maxForce)
        # for steer in self.steeringLinks:
        #     self._p.setJointMotorControl2(self.racecarUniqueId, steer, self._p.POSITION_CONTROL,
        #                                   targetPosition=steeringAngle)

        '''
        (3, b'anki_vector::anki_vector::robot::wheel_BL_hinge'
        (4, b'anki_vector::anki_vector::robot::wheel_BR_hinge'
        (5, b'anki_vector::anki_vector::robot::wheel_FL_hinge'
        (6, b'anki_vector::anki_vector::robot::wheel_FR_hinge'
        '''
        wheels = [3, 4, 5, 6]
        turn_wheels = [3, 5]
        back_wheels = [3, 4]
        # wheels = [3, 4]
        right_side_wheels = [4, 6]
        # todo above into reset
        maxForce = 20

        # for wheel in wheels:
        #     p.setJointMotorControl2(self.racecarUniqueId, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity,
        #                             force=maxForce)
        #
        # for wheel in right_side_wheels:
        #     p.setJointMotorControl2(self.racecarUniqueId, wheel, p.VELOCITY_CONTROL, targetVelocity=-targetVelocity,
        #                             force=maxForce)

        if self.discrete_actions:
            if motorCommands == 0:
                # go forward
                print('go forward')
                for wheel in back_wheels:
                    p.setJointMotorControl2(self.racecarUniqueId, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity,
                                            force=maxForce)
                # todo stop after 50-250ms?
                # time.sleep(0.5)
            elif motorCommands == 1:
                # turn left
                print('turn left')
                vel = targetVelocity
                for idx, wheel in enumerate(turn_wheels):
                    p.setJointMotorControl2(self.racecarUniqueId, wheel, p.VELOCITY_CONTROL,
                                            targetVelocity=vel,
                                            force=maxForce)
                    vel = -targetVelocity
                # time.sleep(0.5)
            elif motorCommands == 2:
                # turn right
                print('turn right')
                vel = -targetVelocity
                for wheel in turn_wheels:
                    p.setJointMotorControl2(self.racecarUniqueId, wheel, p.VELOCITY_CONTROL,
                                            targetVelocity=vel,
                                            force=maxForce)
                    vel = targetVelocity

                # time.sleep(0.5)
