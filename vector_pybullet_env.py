import os
import math
import time

import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet
from agent_env_classes.obj_physics_models import Racecar, Vector
import random
import pybullet_utils.bullet_client as bullet_client
import pybullet_data
from pkg_resources import parse_version
import cv2


class VectorBulletEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self,
                 urdfRoot=pybullet_data.getDataPath(),
                 actionRepeat=1,
                 isEnableSelfCollision=True,
                 isDiscrete=False,
                 renders=False,
                 point_goal=True,
                 rgb_camera_state=False,
                 depth_camera_state=False):
        self._timeStep = 0.01
        self._urdfRoot = urdfRoot
        self._actionRepeat = actionRepeat
        self._isEnableSelfCollision = isEnableSelfCollision
        self._observation = []
        self._ballUniqueId = -1
        self._envStepCounter = 0
        self._renders = renders
        self._point_goal=point_goal
        self._depth_camera_state = depth_camera_state
        self._rgb_camera_state = rgb_camera_state
        self._isDiscrete = isDiscrete
        self._cam_dist = 4
        self._cam_yaw = 0
        self._cam_pitch = 0

        if self._renders:
            self._p = bullet_client.BulletClient(
                connection_mode=pybullet.GUI)
        else:
            self._p = bullet_client.BulletClient()

        self.seed()
        observationDim = 2  # len(self.getExtendedObservation())  # todo make variable and do right
        print("observationDim")
        print(observationDim)  # todo change but now it's a dict/list
        # observation_high = np.array([np.finfo(np.float32).max] * observationDim)
        observation_high = np.ones(observationDim) * 1000  # np.inf
        if (isDiscrete):
            self.action_space = spaces.Discrete(4)
        else:
            # todo continuous as well make sure it works
            action_dim = 2
            self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = spaces.Box(-observation_high, observation_high, dtype=np.float32)
        self.viewer = None

    def reset(self):
        self._p.resetSimulation()
        # p.setPhysicsEngineParameter(numSolverIterations=300)
        self._p.setTimeStep(self._timeStep)
        self._p.loadURDF(os.path.join(self._urdfRoot,"plane.urdf"))
        # stadiumobjects = self._p.loadSDF(os.path.join(self._urdfRoot, "stadium.sdf"))
        # todo remove warnings
        # move the stadium objects slightly above 0
        # for i in stadiumobjects:
        #	pos,orn = self._p.getBasePositionAndOrientation(i)
        #	newpos = [pos[0],pos[1],pos[2]-0.1]
        #	self._p.resetBasePositionAndOrientation(i,newpos,orn)

        dist = 5 + 2. * random.random()
        ang = 2. * 3.1415925438 * random.random()

        ballx = dist * math.sin(ang)
        bally = dist * math.cos(ang)
        ballz = 1

        ballx, bally, ballz = [5, 5, 5]  # fixed goal # todo add option for fixed or random
        # todo add image state feature

        quat_orientation = self._p.getQuaternionFromEuler([0, 0, 3.14 / 2])
        obj_file_name = 'TeaCup.urdf'

        self._ballUniqueId = self._p.loadURDF(obj_file_name, basePosition=[ballx, bally, ballz],
                                              baseOrientation=quat_orientation,
                                              flags=self._p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                                              globalScaling=0.25
                                              )

        # todo reset everything properly?
        self._p.setGravity(0, 0, -10)
        # self._racecar = Racecar(self._p, urdfRootPath=self._urdfRoot,
        #                                 timeStep=self._timeStep)
        self._racecar = Vector(self._p, urdfRootPath=self._urdfRoot,
                               timeStep=self._timeStep)

        self._envStepCounter = 0
        # for i in range(100):  # todo why was this here? to begin things along?
        #     self._p.stepSimulation()
        self._observation = self.getExtendedObservation()
        # return np.array(self._observation)
        return self._observation  # todo dict? or keep as list?

    def __del__(self):
        self._p = 0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        if (self._renders):
            basePos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
            # self._p.resetDebugVisualizerCamera(1, 30, -40, basePos)

        if (self._isDiscrete):
            # fwd = [-1, -1, -1, 0, 0, 0, 1, 1, 1]
            # steerings = [-0.6, 0, 0.6, -0.6, 0, 0.6, -0.6, 0, 0.6]
            # forward = fwd[action]
            # steer = steerings[action]
            # realaction = [forward, steer]

            realaction = action
        else:
            realaction = action

        self._racecar.applyAction(realaction)
        for i in range(self._actionRepeat):
            self._p.stepSimulation()
            if self._renders:
                time.sleep(self._timeStep)
            # todo create many different states:
            # 1. car x,y and orientation,
            # 2. camera rgb
            # 3. camera rgb + depth + segmentation (or variations)
            # 4. car x, y and orientation and goal location as state?
            self._observation = self.getExtendedObservation()

            if self._termination():
                break
            self._envStepCounter += 1
        reward = self._reward()
        done = self._termination()
        # print("len=%r" % len(self._observation))

        # return np.array(self._observation), reward, done, {}
        return self._observation, reward, done, {}  # todo observation as list, dict or array?

    def getExtendedObservation(self):
        self._observation = []  # self._racecar.getObservation()  # todo look at commented function

        if self._rgb_camera_state:
            self._observation.append(self.render(depth=self._depth_camera_state))
        if self._point_goal:
            carpos, carorn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
            ballpos, ballorn = self._p.getBasePositionAndOrientation(self._ballUniqueId)
            invCarPos, invCarOrn = self._p.invertTransform(carpos, carorn)  # todo work out what tihs is
            ballPosInCar, ballOrnInCar = self._p.multiplyTransforms(invCarPos, invCarOrn, ballpos,
                                                                    ballorn)

            self._observation.append([ballPosInCar[0], ballPosInCar[1]])

        if len(self._observation) == 1:  # if only item in list, just create regular numpy array
            self._observation = np.array(self._observation[0])

        return self._observation

    def render(self, mode='rgb_array', close=False, depth=False):
        if mode != "rgb_array":  # 'human' don't return but human can see
            return np.array([])
        base_pos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)

        pixelWidth = 320
        pixelHeight = 220
        camera_height = 0.9

        base_pos = [base_pos[0], base_pos[1], base_pos[2] + camera_height]
        cameraUpVector = [0, 0, 1]
        euler_angle_pose = self._p.getEulerFromQuaternion(orn)
        multiplier = 4.0
        multiplier = 40.0  # todo no or yes or better for far away?
        pitch = euler_angle_pose[0]
        roll = euler_angle_pose[1]
        yaw = euler_angle_pose[2]

        front_vector = [math.cos(yaw) * math.cos(pitch),
                        math.sin(yaw),
                        math.cos(yaw) * math.sin(pitch)]

        front_vector = (np.array(front_vector) / np.linalg.norm(front_vector)).tolist()

        target_position = [base_pos[0] + multiplier * front_vector[0],
                           base_pos[1] + multiplier * front_vector[1],
                           base_pos[2] + multiplier * front_vector[2]]

        viewMatrix = self._p.computeViewMatrix(base_pos, target_position, cameraUpVector)
        projectionMatrix = [1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0,
                            0.0,
                            0.0, -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0]

        (width, height, rgbPixels, depthPixels, segmentationMaskBuffer) = \
            self._p.getCameraImage(pixelWidth, pixelHeight, viewMatrix=viewMatrix,
                                   projectionMatrix=projectionMatrix
                                   )
        # todo? renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(rgbPixels)
        rgb_array = rgb_array[:, :, :3]
        if depth:
            return_array = np.concatenate([rgb_array, np.expand_dims(depthPixels, axis=2)],
                                          axis=2)
        else:
            return_array = rgb_array
        return return_array

    def _termination(self):
        return self._envStepCounter > 1000

    def _reward(self):
        # todo add terminal state option
        closestPoints = self._p.getClosestPoints(self._racecar.racecarUniqueId, self._ballUniqueId,
                                                 10000)

        numPt = len(closestPoints)
        reward = -1000
        # print(numPt)
        if (numPt > 0):
            # print("reward:")
            reward = -closestPoints[0][8]
            # print(reward)
        return reward

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step


if __name__ == '__main__':
    env = VectorBulletEnv(isDiscrete=True, renders=True, point_goal=True, rgb_camera_state=True,
                          depth_camera_state=True)

    for ep in range(5):
        env.reset()
        start = time.time()
        for i in range(10000):
            action = env.action_space.sample()
            state, reward, done, info = env.step(action)

            if i % 100 == 0:
                time_taken = time.time() - start
                time_taken_for_1 = time_taken / 100
                fps = 1 / time_taken_for_1
                print('Time taken to do 100 steps: {}. fps: {}. Reward: {}'.format(time_taken,
                                                                           fps, reward))
                start = time.time()
