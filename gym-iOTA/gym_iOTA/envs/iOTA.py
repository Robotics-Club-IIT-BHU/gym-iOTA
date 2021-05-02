import pybullet as p
import pybullet_data
from math import sqrt
import numpy as np
rnd = np.random.random

class iOTA():
    arena_x = 5             ## Maximum X coordinate of the forseeable arena
    arena_y = 5             ## Maximum Y coordinate of the forseeable arena
    l_wheels = [15, 19]     ## Joint ids of lower left wheels
    r_wheels = [17, 21]     ## Joint ids of lower right wheels
    docks = [25, 11]        ## Joint ids of docking plates
    max_vel = 10            ## This is the maximum velocity a motor can rotate
    min_vel = 1             ## This is the minimum velocity a motor can rotate
    motor_force = 10        ## This is the maximum force a motor can apply
    servo_force = 10        ## This is the maximum force a servo can apply
    dock_encoders = [0,0]   ## This stores the position of servo joint, similar to the real model of a servo

    def __init__(self, path=None, physicsClient=None,position=None,arena=None):
        '''
        Simply initializes One Iota module with the given parameters
        '''
        if path is None:
            path="urdf/iota.urdf"
        self.pClient = physicsClient
        if arena is not None:
            self.arena_x = arena[0]
            self.arena_y = arena[1]
        self.vel = [6*(rnd()-0.5)/0.5, 6*(rnd()-0.5)/0.5]       ## Random initialization of velocity
        theta = np.arctan((self.vel[1]+1e-8)/(self.vel[0]+1e-8))## The yaw of the bot
        self.__setpoint = [0, 0, 0]
        self.__err = 0.01
        if self.vel[0] < 0:
            theta += np.pi
        orie = p.getQuaternionFromEuler((0,0,theta))            ## This is to align the bot to the initialized velocity vector
        basePosition = position or self.init_pos()              ## This is so that it can be respawned in a desired location as well.
        self.id = p.loadURDF(path,
                            basePosition=basePosition,
                            baseOrientation=orie,
                            physicsClientId=self.pClient)       ## Spawning it with the given parameters
        self.dockees = []                                       ## This stores all the docked members to the given bot (Self)
        self.constraints = []                                   ## This stores all the docking constraints

        self.control = self.control_proportional                ## This way we can set the control algorithm we want to use on the bot
        ## Basic Structure of any control algorithm is that it should take a velocity vector towards which the bot should move, and should return True if reached the setpoint
        ## Baseline :- control_proportional
        self.plan = self.plan_vanilla                           ## This way we can set the path planning algorithm we want to use on the bot
        ## Basic Structure of any planning algorithm is that it doesnt take any arguement and return a velocity vector towards the next point on the trajectory
        ## Baseline :- plan_vanilla
        self.get_pos = self.simulator_api_position              ## This way we can set the positioning algorithm that we want to use on the bot
        ## Basic Structure of any planning algorithm is that it takes no arguement and returns (x_coordinate, y_coordinate, z_coordinate ), (x, y, z, w)
        ## Baseline :- simulator_api_position

    def init_pos(self):
        '''
        Random initialization of the bot within the arena limits
        '''
        return [self.arena_x*(rnd()-0.5)/0.5,
                self.arena_y*(rnd()-0.5)/0.5,
                0.001
                ]

    def dist(self, target):
        '''
        Simple function to fetch distance of the bot from a given target point
        '''
        pos = p.getBasePositionAndOrientation(self.id, physicsClientId=self.pClient)[0]
        return sqrt((target[0]-pos[0])**2 + (target[1]-pos[1])**2)

    def low_control(self,vel_arr):
        '''
        This function controls the motor directly with the array of velocities
        Expected input vel_arr = [right_front_wheel_velocity, left_front_wheel_velocity, right_back_wheel_velocity, left_back_wheel_velocity]
        '''
        for i,wheel_set in enumerate([self.r_wheels, self.l_wheels]):               ## Iterating over wheels
            for j,wheel in enumerate(wheel_set):
                p.setJointMotorControl2(self.id,
                                        wheel,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=self.signum_fn(vel_arr[ i + 2*j ] or vel_arr[ i ]),
                                        force=100,
                                        physicsClientId=self.pClient)               ## Applying torque
        return True

    def control_proportional(self,vel_vec):
        '''
        Low level Control algorithm which works on proportional drive with R and theta as there state parameters, it takes in the desired Velocity vector
        '''
        orie = p.getBasePositionAndOrientation(self.id, self.pClient)[1]
        yaw = p.getEulerFromQuaternion(orie)[2]
        set_vec = np.arctan((vel_vec[1]+1e-8)/(vel_vec[0]+1e-8))
        if vel_vec[0]<0:
            set_vec += np.pi
        r = sqrt(vel_vec[0]**2 + vel_vec[1]**2)                                     ## The Magnitude of velocity
        #print(set_vec, yaw)
        theta = set_vec - yaw                                                       ## The amount of angle to be rotated
        ## Please REMEMBER the orientation is inverse hence the sign is inversed
        for_vec = [-np.cos(theta)*r, -np.cos(theta)*r]                                ## Components of Velocity - Forward
        rot_vec = [np.sin(theta)*r, -np.sin(theta)*r]                               ## Components of Velocity - Rotate
        for i,wheel_set in enumerate([self.r_wheels, self.l_wheels]):               ## Iterating over wheels
            for wheel in wheel_set:
                p.setJointMotorControl2(self.id,
                                        wheel,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=for_vec[i]+rot_vec[i],
                                        force=100,
                                        physicsClientId=self.pClient)               ## Applying torque
        self.vel = vel_vec

        if (abs(for_vec[0]+rot_vec[0])+abs(for_vec[1]+rot_vec[1])) < self.__err:    ## check to stop the control algorithm
            return True
        else:
            return False

    def set_point(self, setpoint):
        '''
        This sets the set goal that the planning algorithm will generate the trajectory to this point from current position
        '''
        self.__setpoint = setpoint
        return True

    def plan_vanilla(self):
        '''
        This plans a simple straight line from the current point to the setpoint and doesnt check for obstacle
        '''
        curr_pos, _ = self.get_pos()
        vel_vec = [ 50*(self.__setpoint[i] - curr_pos[i]) for i in range(2) ]
        return vel_vec

    def simulator_api_position(self):
        '''
        This simply returns position of the bot using the pybullet api but other computer vision technique could be inculcated in a similar fashion with IMUs
        '''
        return p.getBasePositionAndOrientation(self.id, self.pClient)

    def signum_fn(self,val):
        '''
        This chops the velocity into a reasonable range of values
        '''
        if val>0:
            if val > self.min_vel:
                return 10*min(self.max_vel,val)
            else:
                return 0
        else:
            if val < -1*self.min_vel:
                return 10*max(-1*self.max_vel, val)
            else:
                return 0

    def set_error(self,error):
        '''
        Sets the tolerable amount of velocity, below which the algorithm is considered completed
        '''
        self.__err = error

    def respawn(self,position=None):
        '''
        Respwans the bot in a new location
        '''
        self.vel = [6*(rnd()-0.5)/0.5, 6*(rnd()-0.5)/0.5]       ## New Random initialization of velocity
        theta = np.arctan(self.vel[1]/self.vel[0]) +np.pi       ## New yaw of the bot
        if self.vel[0] < 0 and self.vel[1]>0:
            theta += np.pi
        orie = p.getQuaternionFromEuler((0,0,theta))            ## New Orientation
        basePosition = position or self.init_pos()              ## Spawns at a given place if passed
        p.resetBasePositionAndOrientation(bodyUniqueId=self.id,
                                          posObj=self.init_pos(),
                                          ornObj=orie,
                                          physicsClientId=self.pClient)

    def dock_servo(self,dock_id,angle):
        '''
        This is a function which emulates the servo in whole. This implements a simple Proportional control try making PD
        '''
        self.dock_encoders[dock_id] = angle                     ## This should not be done but in case of asynchoronous control its helpful
        while True:
            curr = p.getJointState(self.id,
                                   self.docks[dock_id],
                                   self.pClient)[0]             ## Returns the joint position that is the angle rotated
            vel = 10*(angle - curr)                             ## Simple Proportional control
            if vel < 0.05:
                self.dock_encoders[dock_id] = curr
                break
            p.setJointMotorControl2(bodyUniqueId=self.id,
                                    jointIndex=self.docks[dock_id],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=vel,
                                    force=self.servo_force,
                                    physicsClientId=self.pClient)
                                                                ## I have implemented a simple Velocity control, As position control code didnt work
            p.stepSimulate(self.pClient)
            time.sleep(0.01)                                    ## These time rendering should be controlled with a variable as they are on the Main Thread

    def dock(self,other,parent_dock_plate_id=None,child_dock_plate_id=None):
        '''
        This dockes two modules that is self->bot and other->bot
        '''
        ind = None
        try:
            ind = self.dockees.index(other)
        except:
            ind = None
        finally:
            if ind is not None:
                raise Exception(str(self.id)+" and "+str(other.id)+"has a Connection before itself")
                # print(self.id, "->", other.id, "connection exists, redeclaraction!!")
                return -1
        other.dockees.append(self)                              ## Useful to make graph of all nodes
        self.dockees.append(other)                              ## Useful to make graph of all nodes

        pos11, _ = self.get_pos()
        pos22, _ = other.get_pos()

        minn = 1000000000000
        self_id = -1
        other_id = -1
        for i in range(2):
            for j in range(2):
                temp_Self = p.getLinkState(self.id,
                               self.docks[i],
                               )[0]
                temp_other = p.getLinkState(other.id,
                               other.docks[j],
                               )[0]
                temp_dst = sum([(temp_Self[i] - temp_other[i])**2 for i in range(3) ])
                if temp_dst < minn:
                    minn = temp_dst
                    self_id = i
                    other_id = j

        estim = (
                    other.dock_encoders[other_id]           ## The or is so that we could use either of them as mostly both of them must be of the same value
                        or
                    p.getJointState(other.id,
                                    other.docks[other_id],
                                    other.pClient)[0]
                 ) + np.pi                                  ## A phase difference of pi is needed for docking

        self.dock_servo(self_id,estim)

        diff12 = [0, 0, 0]
        diff21 = [0, 0, 0]

        for i in range(3):
            diff12[i] = (pos11[i] - pos22[i]) / 2
            diff21[i] = (pos22[i] - pos11[i]) / 2

        if sum([abs(diff12[i]) for i in range(3)]) > 0.01:
            raise Exception('Please make the bots to come closer before docking')

        cid = p.createConstraint(parentBodyUniqueId=self.id,
                                 parentLinkIndex=-1,            ## THIS SHOULD BE DOCK PLATE INDEX
                                 childBodyUniqueId=other.id,
                                 childLinkIndex=-1,             ## THIS SHOULD BE DOCK PLATE INDEX
                                 jointType=p.JOINT_FIXED,
                                 jointAxis=[0, 0, 0],
                                 parentFramePosition=diff12,
                                 childFramePosition=diff21,
                                 physicsClientId=self.pClient
                                )
        self.constraints.append(cid)
        other.constraints.append(cid)
        return cid

    def undock(self,other):
        '''
        This undocks the two modules that is self->bot and other->bot
        '''
        ind = None
        try:
            ind = self.dockees.index(other)
        except:
            raise Exception(str(self.id)+" "+str(other.id )+", These two modules have not been docked before")
            # print(self.id, other.id, "Are not docked before")
        finally:
            if ind is not None:
                cid = self.constraints.pop(ind)
                other.constraints.remove(cid)
                p.removeConstraint(cid,physicsClientId=self.pClient)
                self.dockees.remove(other)
                other.dockees.remove(self)

    def stop(self):
        '''
        This is the hard hand break for the bot
        '''
        for i,wheel_set in enumerate([self.r_wheels, self.l_wheels]):
            for wheel in wheel_set:
                p.setJointMotorControl2(self.id,
                                        wheel,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=0,
                                        force=self.motor_force,
                                        physicsClientId=self.pClient)

    def __del__(self):
        '''
        This removes the bot out of the simulation
        '''
        for dockee in self.dockees:
            res = self - dockee                                 ## This would undock all the relations and remove on all sides
        p.removeBody(self.id,
                     self.pClient
                     )

    def __add__(self,other):
        '''
        This runs the dock function itself
        '''
        return self.dock(other)

    def __sub__(self,other):
        '''
        This runs the undock function itself
        '''
        self.undock(other)
        return -1
