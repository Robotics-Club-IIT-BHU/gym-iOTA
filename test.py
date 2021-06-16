import gym
import gym_iOTA
import numpy as np
env = gym.make('iOTA-v0',render=True,n=5)
env.reset()
i=0
print(env.action_space)
while i<1000000:
    i+=1
    action = 5*np.ones((env.n,3))
    dock = np.zeros((env.n, env.n))
    env.step(action,dock)
    print('step ',i)
    if i%100000:
        env.render()
env.close()
