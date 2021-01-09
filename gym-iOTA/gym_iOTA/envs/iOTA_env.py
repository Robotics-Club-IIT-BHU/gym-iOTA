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

currentdir = os.path.dirname(
                os.path.abspath(
                            inspect.getfile( inspect.currentframe() )
                                )
                            )
parentdir = os.path.dirname( os.path.dirname(currentdir) )
os.sys.path.insert(0, parentdir)


class IotaEnv(gym.Env):
    metadata = {'render.modes',['human','cluster']}

    def __init__(self, render=False, n=None, no_of_modules=None, k=None, no_of_clusters=None, arena=(2,2), low_control=False):
        '''
        Initializing the env
        '''
        self.pClient = p.connect(p.GUI if render else p.DIRECT)
        self.rend = render
        if self.rend:
            p.resetDebugVisualizerCamera(
                                        cameraDistance=1.5,
                                        cameraYaw=0,
                                        cameraPitch=-40,
                                        cameraTargetPosition=[0.55,-0.35,0.2],
                                        physicsClientId=self.pClient
                                        )
        self.n = (n or no_of_modules) or 10
        self.k = (k or no_of_clusters) or 1
        self.cube = p.loadURDF(currentdir+'/absolute/dabba.urdf',
                                basePosition=(arena[0],0,0.5),
                                physicsClientId=self.pClient)
        self.target_pos = (-arena[0],0,0.5)
        self.low_control = low_control
        if self.low_control:
            self.action_space = spaces.Box(
                                            low=np.array( [ [-6, -6, -6, -6] ]*self.n),
                                            high=np.array( [ [6, 6, 6, 6] ]*self.n )
                                        )
        else:
            self.action_space = spaces.Box(
                                            low=np.array(
                                                [[*(-1*np.array(arena)),-0.5]]*self.n
                                                ),
                                            high=np.array(
                                                [[*np.array(arena),0.5]]*self.n
                                                )
                                            )
        self.observation_space = spaces.Box(
                                        low=np.array(
                                            [[*(-1*np.array(arena)),-0.5]]*self.n
                                            ),
                                        high=np.array(
                                            [[*np.array(arena),0.5]]*self.n
                                            )
                                        )
        self.iotas = [ iOTA(currentdir+"/absolute/iota.urdf",self.pClient) for i in range(self.n) ]

        self.get_pos = lambda inst, x : (
                        p.getBasePositionAndOrientation(x.id,inst.pClient)[0],
                        p.getEulerFromQuaternion(
                                p.getBasePositionAndOrientation(x.id,inst.pClient)[1]
                            )
                    )                                                           ## This is totally depreciated but is a alternative to the multicamera detections and IMUs

    def step(self,action,docks):
        '''
        Simple Step function
        '''
        docks = np.zeros((self.n,self.n)) - np.eye(self.n)                       ## disabling docking to reduce complexity
        if self.low_control:
            for i,(act,iota) in enumerate(zip(*action,self.iotas)):
                iota.low_control(act)
                p.stepSimulation(self.pClient)
        else:
            for i,(pos,iota) in enumerate(zip(*action,self.iotas)):
                iota.set_point(pos)
            final = False
            while not final:
                final = True
                for iota in self.iotas:
                    vec = iota.plan()
                    final = final and iota.control(vec)
                for i in range(5):
                    p.stepSimulation(self.pClient)
        object_pos = p.getBasePositionAndOrientation(self.cube, self.pClient)[0]
        reward = -sum(abs(object_pos[i] - self.target_pos[i]) for i in range(3))
        observation = []
        for iota in self.iotas:
            observation.append(self.get_pos(iota)[0])
        observation = np.array(observation)
        done = reward > 0.05*self.arena[0]
        return observation, reward, done, {}

    def reset(self):
        '''
        Resets the env
        '''
        pass

    def render(self):
        '''
        Renders the scene if allowed
        '''

        pass

    def close(self):
        '''
        Simply closes the connection to the physics server
        '''
        p.disconnect(self.pClient)
