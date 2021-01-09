import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data
import numpy as np
import cv2
from iOTA import iOTA

import pkg_resources

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


class IotaEnv(gym.Env):
    metadata = {'render.modes',['human','cluster']}

    def __init__(self,render=False,n=None,no_of_modules=None,k=None,no_of_clusters=None,arena=(1,1)):
        self.pClient = p.connect(p.GUI if render else p.DIRECT)
        self.rend = render
        if self.rend:
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,cameraTargetPosition=[0.55,-0.35,0.2],self.pClient)
        self.n = (n or no_of_modules) or 10
        self.k = (k or no_of_clusters) or 1
        self.action_space = spaces.Box(low=np.array([[*(-1*np.array(arena)),-0.5]]*self.n),high=np.array([[*np.array(arena),0.5]]*self.n))
        self.observation_space = spaces.Box(low=np.array([[*(-1*np.array(arena)),-0.5]]*self.n),high=np.array([[*np.array(arena),0.5]]*self.n))
        self.iotas = [ iOTA(currentdir+"/absolute/iota.urdf",self.pClient) for i in range(self.n) ]
        ## Replacing the pointer with an custom function like for multicamera will work fine
        self.get_pos = lambda inst,x : (p.getBasePositionAndOrientation(x.id,inst.pClient)[0],p.getEulerFromQuaternion(p.getBasePositionAndOrientation(x.id,inst.pClient)[1]))
    def step(self,action):
        for i,(pos,iota) in enumerate(zip(*action,self.iotas)):
            iota.set_point(pos)
        final = False
        while not final:
            final = True
            for iota in self.iotas:
                vec = iota.plan()
                final = final and iota.control()
            for i in range(5):
                p.stepSimulation(self.pClient)
        reward = 0
        observation = []
        for iota in self.iotas:
            observation.append(self.get_pos(iota))
        observation = np.array(observation)
        return observation, reward, False, {}
    def reset(self):
        pass

    def render(self):
        pass

    def close(self):
        p.disconnect(self.pClient)
