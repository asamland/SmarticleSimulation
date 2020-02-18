import pybullet as p
import time
import pybullet_data
from simulation_smarticle import SimulationSmarticle as ss
from transforms import *

from pdb import set_trace as bp

import numpy as np

#URDF paths
urdf_path = '../urdf/smarticle.urdf'
ring_path = '../urdf/ring.urdf'

def time_to_steps(time_s):
    return int(time_s*240)



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
maxvel = 6.
dt = time_to_steps(0.45)
x = 0.02
r = p.loadURDF(ring_path, basePosition = [0,0,0])
cid = p.createConstraint(r,-1,0,-1,p.JOINT_PRISMATIC,[0,0,0],[0,0,0],[0,0,0])
dx = 0.032
th = np.pi/2
s1 = ss(p, urdf_path, maxvel, basePosition = [0,-2*dx,0], baseOrientation = [0,0,th])
s2 = ss(p, urdf_path, maxvel, basePosition = [0,-dx,0], baseOrientation = [0,0,th])
s3 = ss(p, urdf_path, maxvel, basePosition = [0,0,0], baseOrientation = [0,0,th])
s4 = ss(p, urdf_path, maxvel, basePosition = [0,dx,0], baseOrientation = [0,0,th])
s5 = ss(p, urdf_path, maxvel, basePosition = [0,2*dx,0], baseOrientation = [0,0,th])
bp()
smarticles = [s1]

smarticles = [s1, s2, s3, s4]

R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]
for s in smarticles:
    s.load_gait(np.array([L,R]),dt)

for i in range (480):
    p.stepSimulation()

test = p.getDynamicsInfo(s1.id,0)
bp()
t_steps = time_to_steps(1200)
for i in range (t_steps):
    p.stepSimulation()
    time.sleep(1./240.)
    if i%dt==s1.gait_phase:
        s1.motor_step()
        # s1.update_position()
        # bp()
    if i%dt==s2.gait_phase:
        s2.motor_step()
    if i%dt==s3.gait_phase:
        s3.motor_step()
    if i%dt==s4.gait_phase:
        s4.motor_step()
    # if i%dt==s5.gait_phase:
    #     s5.motor_step()
p.disconnect()
