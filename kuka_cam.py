import cv2

import pybullet_envs.bullet.kukaCamGymEnv as e


env = e.KukaCamGymEnv(renders=True)
env.reset()

for i in range(2500):
    #image = env.render()
    action = env.action_space.sample()
    state, reward, done, info = env.step(action)

    #cv2.imshow('Render', image)
    #print(i)