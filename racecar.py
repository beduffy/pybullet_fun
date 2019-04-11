import pybullet_envs.bullet.racecarGymEnv as e
env = e.RacecarGymEnv(isDiscrete=False ,renders=True)
env.reset()

for i in range(1000):
    #image = env.render()
    action = env.action_space.sample()
    state, reward, done, info = env.step(action)