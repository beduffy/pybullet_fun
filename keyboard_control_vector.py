import os, inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import gym
import time
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet
from agent_env_classes.obj_physics_models import Racecar, Vector
import random
import pybullet_utils.bullet_client as bullet_client
import pybullet_data
from pkg_resources import parse_version
import pybullet as p

RENDER_HEIGHT = 720
RENDER_WIDTH = 960

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
                 renders=False):
        print("init")
        self._timeStep = 0.01
        self._urdfRoot = urdfRoot
        self._actionRepeat = actionRepeat
        self._isEnableSelfCollision = isEnableSelfCollision
        self._observation = []
        self._ballUniqueId = -1
        self._envStepCounter = 0
        self._renders = renders
        self._isDiscrete = isDiscrete
        if self._renders:
            self._p = bullet_client.BulletClient(
                connection_mode=pybullet.GUI)
        else:
            self._p = bullet_client.BulletClient()
        # self._p = bullet_client.BulletClient()

        self.seed()
        # self.reset()
        observationDim = 2  # len(self.getExtendedObservation())
        # print("observationDim")
        # print(observationDim)
        # observation_high = np.array([np.finfo(np.float32).max] * observationDim)
        observation_high = np.ones(observationDim) * 1000  # np.inf
        if (isDiscrete):
            self.action_space = spaces.Discrete(4)
        else:
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
        # self._p.loadURDF(os.path.join(self._urdfRoot,"plane.urdf"))
        stadiumobjects = self._p.loadSDF(os.path.join(self._urdfRoot, "stadium.sdf"))
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

        ballx, bally, ballz = [5, 5, 5]
        ballx, bally, ballz = [0.5, 0.5, 2]

        quat_orientation = self._p.getQuaternionFromEuler([1, 0, 0])

        cup_scale = 0.01
        # obj_file_name = 'coffee_cup.obj'
        # obj_file_name = 'TeaCup.obj'
        obj_file_name = 'TeaCup.urdf'

        self._ballUniqueId = self._p.loadURDF(obj_file_name, basePosition=[ballx, bally, ballz],
                                              baseOrientation=quat_orientation,
                                              flags=self._p.URDF_USE_MATERIAL_COLORS_FROM_MTL
                                              )

        ballx, bally, ballz = [-0.5, -0.5, 0.5]
        obj_file_name = 'coffee_cup.urdf'
        self._secondCupUniqueId = self._p.loadURDF(obj_file_name, basePosition=[ballx, bally, ballz],
                                              baseOrientation=quat_orientation,
                                              flags=self._p.URDF_USE_MATERIAL_COLORS_FROM_MTL
                                              )

        range_in_each_dim = 5
        for i in range(1):
            ballx, bally, ballz = [random.uniform(-range_in_each_dim, range_in_each_dim),
                                   random.uniform(-range_in_each_dim, range_in_each_dim),
                                   random.uniform(0.3, 0.3)]
            # obj_file_name = random.choice(['coffee_cup.urdf', 'TeaCup.urdf'])
            # obj_file_name = random.choice(['TeaCup.urdf'])
            obj_file_name = random.choice(['coffee_cup.urdf'])
            _secondCupUniqueId = self._p.loadURDF(obj_file_name,
                                                       basePosition=[ballx, bally, ballz],
                                                       baseOrientation=quat_orientation,
                                                       # rgbaColor=[1, 0, 0, 1]
                                                       flags=self._p.URDF_USE_MATERIAL_COLORS_FROM_MTL
                                                       )
            self._p.changeVisualShape(_secondCupUniqueId, 0,
                                      # rgbaColor=[random.uniform(0, 1), random.uniform(0, 1),
                                      #            random.uniform(0, 1), 1]
                                      rgbaColor=[random.randint(0, 1), random.randint(0, 1),
                                                 random.randint(0, 1), 1]
                                      )

        # self._ballUniqueId = self._p.loadURDF(os.path.join(self._urdfRoot, "sphere2.urdf"),
        # [ballx, bally, ballz])
        # self._ballUniqueId = self._p.loadSDF(os.path.join(self._urdfRoot, "teacup_model.sdf"))
        # self._ballUniqueId = self._p.loadSDF("teacup_model.sdf", globalScaling=0.5)
        # self._ballUniqueId = self._ballUniqueId[0]  # array of links

        # self._ballUniqueIdCol = self._p.createCollisionShape(self._p.GEOM_CYLINDER,
        # # self._ballUniqueId = self._p.createCollisionShape(self._p.GEOM_CYLINDER,
        #                                                      fileName=obj_file_name,
        #                                                      meshScale=[cup_scale, cup_scale,
        #                                                                 cup_scale])
        # self._ballUniqueId = self._p.createVisualShape(self._p.GEOM_MESH,
        #                                                fileName=os.path.join(self._urdfRoot,
        #                                                                      obj_file_name),
        #                                                # rgbaColor=[1, 0, 0, 1],
        #                                                meshScale=[cup_scale, cup_scale, cup_scale])
        #
        # self._p.createMultiBody(baseMass=1,
        #                         #baseInertialFramePosition=[0, 0, 0],
        #                   # baseCollisionShapeIndex=self._ballUniqueId,
        #                   baseCollisionShapeIndex=self._ballUniqueIdCol,
        #                   baseVisualShapeIndex=self._ballUniqueId,
        #                   # basePosition=[((-rangex / 2) + i) * meshScale[0] * 2,
        #                   #               (-rangey / 2 + j) * meshScale[1] * 2, 1],
        #                   basePosition = [3, 3, 3],
        #                   useMaximalCoordinates=True)



        # todo reset everything properly?
        self._p.setGravity(0, 0, -10)
        # self._racecar = Racecar(self._p, urdfRootPath=self._urdfRoot,
        #                                 timeStep=self._timeStep)
        self._racecar = Vector(self._p, urdfRootPath=self._urdfRoot,
                               timeStep=self._timeStep)
        self._envStepCounter = 0
        # for i in range(100):  # todo why was this here?
        #     self._p.stepSimulation()
        self._observation = self.getExtendedObservation()
        return np.array(self._observation)

    def __del__(self):
        self._p = 0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def getExtendedObservation(self):
        self._observation = []  # self._racecar.getObservation()
        carpos, carorn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
        ballpos, ballorn = self._p.getBasePositionAndOrientation(self._ballUniqueId)
        invCarPos, invCarOrn = self._p.invertTransform(carpos, carorn)
        ballPosInCar, ballOrnInCar = self._p.multiplyTransforms(invCarPos, invCarOrn, ballpos,
                                                                ballorn)

        self._observation.extend([ballPosInCar[0], ballPosInCar[1]])
        return self._observation

    def step(self, action):
        if (self._renders):
            basePos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
            # self._p.resetDebugVisualizerCamera(1, 30, -40, basePos)

        # import pdb;pdb.set_trace()
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
            self._observation = self.getExtendedObservation()

            if self._termination():
                break
            self._envStepCounter += 1
        reward = self._reward()
        done = self._termination()
        # print("len=%r" % len(self._observation))

        return np.array(self._observation), reward, done, {}

    def render(self, mode='human', close=False):
        if mode != "rgb_array":
            return np.array([])
        base_pos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=self._cam_dist,
            yaw=self._cam_yaw,
            pitch=self._cam_pitch,
            roll=0,
            upAxisIndex=2)
        proj_matrix = self._p.computeProjectionMatrixFOV(
            fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
            nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = self._p.getCameraImage(
            width=RENDER_WIDTH, height=RENDER_HEIGHT, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _termination(self):
        return self._envStepCounter > 1000

    def _reward(self):
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

    # renders = True
    # # env = VectorBulletEnv(isDiscrete=False, renders=True)
    # # env = VectorBulletEnv(isDiscrete=True, renders=True)
    # env = VectorBulletEnv(isDiscrete=True, renders=renders)
    #
    # num_repeat = 200
    # action_arr = [1] * num_repeat + [0] * num_repeat + [2] * num_repeat + [3] * num_repeat
    # for ep in range(5):
    #     env.reset()
    #     # for action in action_arr:
    #     start = time.time()
    #     for i in range(100000):
    #         # image = env.render()
    #         action = env.action_space.sample()
    #         state, reward, done, info = env.step(action)
    #
    #         if i % 100 == 0:
    #             time_taken = time.time() - start
    #             time_taken_for_1 = time_taken / 100
    #             fps = 1 / time_taken_for_1
    #             print('Time taken to do 100 steps: {}. For one: {}. fps: {}'.format(time_taken,
    #                                                                        time_taken_for_1,
    #                                                                        fps))
    #             start = time.time()

    pixelWidth = 320
    pixelHeight = 220
    # camDistance = 4
    # camDistance = 0.5
    # upAxisIndex = 2
    camera_height = 0.9

    cid = p.connect(p.SHARED_MEMORY)
    if (cid < 0):
        p.connect(p.GUI)

    p.resetSimulation()
    p.setGravity(0, 0, -10)

    stadiumobjects = p.loadSDF("stadium.sdf")

    vector = Vector(p)

    targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 50, 0)
    maxForceSlider = p.addUserDebugParameter("maxForce", 0, 50, 20)
    steeringLeftSlider = p.addUserDebugParameter("steeringleft", -1, 1, 0)
    steeringRightSlider = p.addUserDebugParameter("steeringright", -1, 1, 0)
    t = 0
    value_to_add = 0.0
    while (True):
        maxForce = p.readUserDebugParameter(maxForceSlider)
        targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        steeringLeftVel = p.readUserDebugParameter(steeringLeftSlider)
        steeringRightVel = p.readUserDebugParameter(steeringRightSlider)

        # for wheel in wheels:
        #     p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity,
        #                             force=maxForce)
        #
        # for wheel in right_side_wheels:
        #     p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=-targetVelocity,
        #                             force=maxForce)

        # for steer in steering:
        #     p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=-steeringAngle)

        # for steer in steering:
        # p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL, targetVelocity=targetVelocity + steeringLeftVel)
        # p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL, targetVelocity=targetVelocity + steeringRightVel)

        # if t % 200 == 0:
        car_orientation = p.getBasePositionAndOrientation(vector.racecarUniqueId)
        x_y_z = list(car_orientation[0])
        x_y_z[2] += camera_height
        quaternion_pose = car_orientation[1]
        # print()
        cameraUpVector = [0, 0, 1]
        # target_position = [0, 0, 0]
        euler_angle_pose = p.getEulerFromQuaternion(quaternion_pose)
        multiplier = 4.0
        multiplier = 40.0
        pitch = euler_angle_pose[0]
        roll = euler_angle_pose[1]
        yaw = euler_angle_pose[2]

        front_vector = [math.cos(yaw) * math.cos(pitch),
                        math.sin(yaw),
                        math.cos(yaw) * math.sin(pitch)]

        front_vector = (np.array(front_vector) / np.linalg.norm(front_vector)).tolist()

        target_position = [x_y_z[0] + multiplier * front_vector[0],
                           x_y_z[1] + multiplier * front_vector[1],
                           x_y_z[2] + multiplier * front_vector[2]]

        viewMatrix = p.computeViewMatrix(x_y_z, target_position, cameraUpVector)

        projectionMatrix = [1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0,
                            0.0,
                            0.0, -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0]

        img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix=viewMatrix,
                                   projectionMatrix=projectionMatrix,
                                   shadow=1,
                                   lightDirection=[1, 1, 1]
                                   )

        # todo PyBullet.isNumpyEnabled(

        keys = p.getKeyboardEvents()
        print(keys)
        if keys.get(65297):  # up
            vector.applyAction(0)
        if keys.get(65295):  # left
            vector.applyAction(1)
        if keys.get(65296):  # right
            vector.applyAction(2)
        if keys.get(65298):  # down
            vector.applyAction(3)

        p.stepSimulation()
        # time.sleep(0.01)


        t += 1

