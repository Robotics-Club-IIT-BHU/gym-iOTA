from iOTA import iOTA
import pybullet as p
import numpy as np
import pybullet_data
import time
pClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.setGravity(0,0,-10)
z=0.001
o_n = 24
O = [ [ 0.6*np.sin(np.pi*i*2 / o_n) - 1 , np.cos(np.pi*i*2/ o_n), z ] for i in range(o_n)]

iotas_o = [ iOTA(path='absolute/iota.urdf',position=pos_o,physicsClient=pClient) for pos_o in O ]

i_n = 7
I = [ [-2,0.2*(i-5),z] for i in range(i_n) ]
iotas_i = [ iOTA(path='absolute/iota.urdf',position=pos_i,physicsClient=pClient) for pos_i in I ]
I_tip = [-2, 0.85 ,z]
iota_i_tip = iOTA(path='absolute/iota.urdf',position=I_tip,physicsClient=pClient)

t1_n = 9
t2_n = 10
T1 = [ [1,0.2*(i-5),z] for i in range(t1_n) ]
iotas_t1 = [ iOTA(path='absolute/iota.urdf',position=pos_t,physicsClient=pClient) for pos_t in T1 ]
T2 = [ [1+0.2*(i-t2_n/2), 1, z] for i in range(t2_n)]
iotas_t2 = [ iOTA(path='absolute/iota.urdf',position=pos_t,physicsClient=pClient) for pos_t in T2 ]

a1_n = 10
A1 = [[2.2+0.06*(i-5),0.2*(i-5),z] for i in range(a1_n)]
iotas_a1 = [ iOTA(path='absolute/iota.urdf',position=pos_a,physicsClient=pClient) for pos_a in A1 ]
a2_n = 9
A2 = [[2.8-0.06*(i-5),0.2*(i-5),z] for i in range(a2_n)]
iotas_a2 = [ iOTA(path='absolute/iota.urdf',position=pos_a,physicsClient=pClient) for pos_a in A2 ]
a3_n = 3
A3 = [ [2.5 + 0.2*(i-1.5),0,z] for i in range(a3_n) ]
iotas_a3 = [ iOTA(path='absolute/iota.urdf',position=pos_a,physicsClient=pClient) for pos_a in A3 ]

tip = [-5, 0.85, z]
iota_i_tip.set_error(100)
iota_i_tip.set_point(tip)
time.sleep(6)
vec = iota_i_tip.plan()
vec[0] *=50; vec[1] *= 50
while not iota_i_tip.control(vec) :
    vec = iota_i_tip.plan()
    vec[0] *=50; vec[1] *=50
    for i in range(10):p.stepSimulation(pClient)
    time.sleep(0.005)
while True:
    for iotas in [iotas_i, iotas_o, iotas_t1, iotas_t2, iotas_a1, iotas_a2, iotas_a3]:
        for iota in iotas:
            iota.control(iota.vel)
    for i in range(10):p.stepSimulation(pClient)
    time.sleep(0.005)
