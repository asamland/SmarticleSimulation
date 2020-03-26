import pybullet as p
import os
import time
import pybullet_data
from datetime import datetime
import numpy as np

import smarticlesimulation as sim
from smarticlesimulation.flashlight import Flashlight

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle

from pdb import set_trace as bp



def init_axes():

    # plt.axis('off')
    # plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')
    ax.axis([-110, 110, -110, 110,])
    ring_radius = 100
    circle1 = Circle((0,0), ring_radius, color='black', fill=False)
    ax.add_artist(circle1)

    return ax


def draw_rectangle(ax,xy, th, color):
    width = 54
    height = 30
    rect_c =(0.749,0.624,1)
    ang= 90+180/np.pi*th
    ax.add_patch(Rectangle(xy=xy ,width=width, height=height, angle = ang,\
                                linewidth=0, color=color, alpha=0.002))
    # plt.pause(0.0001)

GUI = False

#URDF paths
urdf_dir = '../urdf/'
smarticle_path = urdf_dir+'smarticle.urdf'
ring_path = urdf_dir+'ring.urdf'
path_80_20 = urdf_dir+'80_20.urdf'
table_path = urdf_dir+'table/table.urdf'
flashlight_path = urdf_dir+'flashlight.urdf'

key_fun = lambda x: x[0]
c = ('red','yellow','blue','white')



server = p.GUI if GUI else p.DIRECT
physicsClient = p.connect(server)#or p.DIRECT for non-graphical version
runs = 20
total_displacement = np.zeros(runs)
z = 0.6
fl = Flashlight(flashlight_path,[0,1,z+0.025], 3*np.pi/2)
fl.set_polar_position([0,0],r=1,th=np.pi/2)
n = 5
R = [-1.7,1.7,1.7,-1.7]
L = [1.7,1.7,-1.7,-1.7]
dt = sim.time_to_steps(0.45)
time_s = 8*60
t_steps = sim.time_to_steps(time_s)
t0 = time.time()



p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF(table_path)
table_constraint_id = p.createConstraint(tableId,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
# wall1 = p.loadURDF(path_80_20, basePosition=[-0.2,0.25,z])
# p1c_id = p.createConstraint(wall1,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
# wall2 = p.loadURDF(path_80_20, basePosition=[0.2,0.25,z])
# p2c_id = p.createConstraint(wall2,-1,0,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
r = p.loadURDF(ring_path, basePosition = [0,0,z])
smarticles = sim.load_smarticles(n,smarticle_path,[L,R],dt,z)
for iter in range(runs):
    ax = init_axes()
    plt.title("Run {}".format(1+iter))
    p.resetBasePositionAndOrientation(r,[0,0,z],p.getQuaternionFromEuler([0,0,0]))
    ring_constraint_id = p.createConstraint(r,-1,-1,-1,\
                                p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,z])
    for s in smarticles:
        s.reset_pose()
    #for i in range (240*1):
    for i in range(1):
        p.stepSimulation()
        for s in smarticles:
            if i%dt==s.gait_phase:
                s.move_random_corners()
    p.removeConstraint(ring_constraint_id)
    plot_ii = 0
    for i in range (t_steps):
        p.stepSimulation()
        # time.sleep(1./240.)
        if i%40==0:
            pos,_ = p.getBasePositionAndOrientation(r)
            fl.set_polar_position(pos[:2])
            fl.ray_check(smarticles)
            if plot_ii %8==0: #plot every 1s
                states = []
                for s in smarticles:
                    ring_center = np.array(pos[:2])
                    if s.plank ==1:
                        state = np.zeros(3)
                        state[:2]=1000*(s.x[:2]-ring_center)
                        state[2] = s.x[2]
                        states.append(state)
                    if len(states)==1:
                        state = states[0]
                        draw_rectangle(ax, state[:2], state[2],c[0])
                    elif len(states)>1 and len(states)<4:
                        states.sort(key=key_fun)
                        for c_ii, state in enumerate(states):
                            draw_rectangle(ax, state[:2], state[2],c[c_ii+1])

            plot_ii+=1
        for s in smarticles:
            if i%dt==s.gait_phase:
                s.motor_step()
                #s.move_random_corners()
    p.removeAllUserDebugItems()
    pos, _ = p.getBasePositionAndOrientation(r)
    total_displacement[iter]=int(1000*pos[1])
    plt.text(-110,100, "dX: {}".format(total_displacement[iter]))
    plt.axis('off')
    plt.show(block = False)
    plt.pause(1)
    print('\n\ntime:{}, run:{}, displacement:{}'.format(time.time()-t0,iter+1,total_displacement[iter]))
p.disconnect()
print('Average Displacement: {}'.format(sum(total_displacement)/runs))
print("Done!")
