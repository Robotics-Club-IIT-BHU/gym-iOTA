import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import inspect
import pybullet as p
import pybullet_data
import numpy as np
import cv2
from PIL import Image
from .iOTA import iOTA

import pkg_resources

currentdir = os.path.dirname(
                os.path.abspath(
                            inspect.getfile( inspect.currentframe() )
                                )
                            )
parentdir = os.path.dirname( os.path.dirname(currentdir) )
os.sys.path.insert(0, parentdir)


class IotaEnv(gym.Env):
    metadata = {'render.modes':['human','cluster']}

    def __init__(self, render=False, n=None, no_of_modules=None, k=None, no_of_clusters=None, arena=(2,2), low_control=True):
        '''
        Initializing the env
        '''
        self.pClient = -1
        self.rend = render
        self.n = (n or no_of_modules) or 10
        self.no_of_modules = self.n
        self.k = (k or no_of_clusters) or 1
        self.no_of_clusters = self.k
        self.arena = arena
        
        self.target_pos = (-arena[0],0,0.5)
        self.low_control = low_control
        if self.low_control:
            self.action_space = spaces.Box(
                                            low=np.array( [ [-6, -6, -6, -6] ]*self.n ,dtype=np.float32),
                                            high=np.array( [ [6, 6, 6, 6] ]*self.n , dtype=np.float32)
                                        )
        else:
            self.action_space = spaces.Box(
                                            low=np.array(
                                                [[*(-1*np.array(arena)),-0.5]]*self.n, dtype=np.float32
                                                ),
                                            high=np.array(
                                                [[*np.array(arena),0.5]]*self.n, dtype=np.float32
                                                )
                                            )
        self.observation_space = spaces.Box(
                                        low=np.array(
                                            [[*(-1*np.array(arena)),-0.5]]*self.n , dtype=np.float32
                                            ),
                                        high=np.array(
                                            [[*np.array(arena),0.5]]*self.n , dtype = np.float32
                                            )
                                        )
        self.dockingMatrix = np.zeros((self.n, self.n))

    def setup_env(self):
        self.pClient = p.connect(p.GUI if self.rend else p.DIRECT)
        p.setGravity(0,0,-10,physicsClientId=self.pClient)
        if self.rend:
            p.resetDebugVisualizerCamera(
                                        cameraDistance=1.5,
                                        cameraYaw=0,
                                        cameraPitch=-40,
                                        cameraTargetPosition=[0.55,-0.35,0.2],
                                        physicsClientId=self.pClient
                                        )
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = p.loadURDF('plane.urdf',
                                physicsClientId=self.pClient)
        self.cube = p.loadURDF(currentdir+'/absolute/dabba.urdf',
                                basePosition=(arena[0]/2,0,0.1),
                                physicsClientId=self.pClient)

        self.iotas = [ iOTA(path=currentdir+"/absolute/iota.urdf",physicsClient=self.pClient,arena=self.arena) for i in range(self.n) ]

        self.get_pos = lambda x : (
                        p.getBasePositionAndOrientation(x.id,self.pClient)[0],
                        p.getEulerFromQuaternion(
                                p.getBasePositionAndOrientation(x.id,self.pClient)[1]
                            )
                    )                                                           ## This is totally depreciated but is a alternative to the multicamera detections and IMUs
        


    def step(self,action,docks):
        '''
        Simple Step function
        '''
        reward = 0
        docks = np.zeros((self.n,self.n)) - np.eye(self.n)                      ## disabling docking to reduce complexity
        if self.low_control:
            for i,(act,iota) in enumerate(zip(action,self.iotas)):
                for j,dock in enumerate(docks[i,:]):
                    if dock>=1:
                        try:
                            iota.dock(self.iotas[j])                            ## Try because the bots if apart by a big distance its not possible to dock
                        except:
                            reward -= 1                                         ## This is for punishing the policy to have choosen to dock
                iota.low_control(act)
                p.stepSimulation(self.pClient)
        else:
            for i,(pos,iota) in enumerate(zip(action,self.iotas)):
                iota.set_point(pos)
                for j,dock in enumerate(docks[i,:]):
                    if dock>=1:
                        try:
                            iota.dock(self.iotas[j])                            ## Try because the bots if apart by a big distance its not possible to dock
                        except:
                            reward -= 1                                         ## This is for punishing the policy to have choosen to dock
            final = False
            while not final:
                final = True
                for iota in self.iotas:
                    vec = iota.plan()
                    final = final and iota.control(vec)
                for i in range(5):
                    p.stepSimulation(self.pClient)
        object_pos = p.getBasePositionAndOrientation(self.cube, self.pClient)[0]
        reward += -sum(abs(object_pos[i] - self.target_pos[i]) for i in range(3))
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
        if self.pClient<0:
            self.setup_env()
        self.dockingMatrix = np.zeros((self.n, self.n))
        p.resetBasePositionAndOrientation(
            self.cube,
            posObj=(self.arena[0]/2, 0, 0.1),
            ornObj=(0, 0, 0, 1),
            physicsClientId=self.pClient
        )
        for iota in self.iotas:
            for dockee in iota.dockees:
                iota.undock(dockee)
            #iota.respawn()
        observation = []
        for iota in self.iotas:
            observation.append(self.get_pos(iota)[0])
        observation = np.array(observation)
        return observation

    def render(self,mode='cluster'):
        '''
        Renders the scene if allowed
        '''
        img = self.get_top_view()
        if self.rend or mode=='human':                          ## so that for debug on a local gui based system you can see the output
            im = Image.fromarray(img[:,:,[2,1,0]])
            im.show()
        return img


    def get_top_view(self):
        '''
        Returns the top view of the whole environment
        '''
        viewMatrix = p.computeViewMatrix(
                                        cameraEyePosition=(0,0,1.5*self.arena[0]),
                                        cameraTargetPosition=(0,0,0),
                                        cameraUpVector=(1,0,0),
                                        physicsClientId=self.pClient
                                        )
        projectionMatrix = p.computeProjectionMatrixFOV(
                                        fov=60,
                                        aspect=1,
                                        nearVal=0.1*self.arena[0],
                                        farVal=1.8*self.arena[0],
                                        physicsClientId=self.pClient
                                        )
        return p.getCameraImage(width=400,
                                height=400,
                                viewMatrix=viewMatrix,
                                projectionMatrix=projectionMatrix,
                                renderer= p.ER_BULLET_HARDWARE_OPENGL if self.rend else p.ER_TINY_RENDERER,
                                physicsClientId=self.pClient)[2]

    def close(self):
        '''
        Simply closes the connection to the physics server
        '''
        for iota in self.iotas:
            del iota
        p.disconnect(self.pClient)
