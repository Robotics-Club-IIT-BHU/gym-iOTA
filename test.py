import gym
import gym_iOTA
import numpy as np
env = gym.make('iOTA-v0',render=True,n=30)

i=0

while i<1000000:
    i+=1
    action = np.ones((4,env.n))
    dock = np.zeros((env.n, env.n))
    env.step(action,dock)
    if i%100000:
        env.render()
env.close()
