# -*- coding: utf-8 -*-
"""
Created on Fri Dec 16 18:34:56 2016
@author: pangh

Reimplementation of Reciprocal Velocity Obstacle(RVO) algorithm;
Extend 2D case to 3D;
Plot 3D trajectories
"""

from RVO_3d import RVO_update_3d, compute_V_des_3d, distance_3d
#from vis import visualize_traj_dynamic


#------------------------------
#define workspace model
ws_model = dict()
#robot radius
# In our case safety region
ws_model['robot_radius'] = 0.2
## no obstacles
#ws_model['circular_obstacles'] = []
##rectangular boundary, format [x,y,width/2,heigth/2]
## TODO: Later we may want to set 3D boundaries, but hopefully they won't hit the wall
#ws_model['boundary'] = [] 

#------------------------------
#initialization for robot 
# position of [x,y,z]
num=7
X = [[-0.5+1.0*i, 0.0, 0.0] for i in range(num)] + [[-0.5+1.0*i, 5.0, 1.0] for i in range(num)]
# velocity of [vx,vy,vz]
# Default as zero vectors
V = [[0,0,0] for i in xrange(len(X))]
# maximal velocity norm
# TODO: What is default velocity for our Position Handler?
V_max = [1.0 for i in xrange(len(X))]
# goal of [x,y,z]
goal = [[5.5-1.0*i, 5.0, 1.0] for i in range(num)] + [[5.5-1.0*i, 0.0, 0.0] for i in range(num)]

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 10
# simulation step
step = 0.05
#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des_3d(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update_3d(X, V_des, V, ws_model)
    # update position
    residue = 0
    for i in xrange(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
        X[i][2] += V[i][2]*step
        print 'z='+str(X[i][2])
        residue += distance_3d(X[i],goal[i])
    print "residue"+str(residue)
    print "-----------------------"
#    #----------------------------------------
#    # visualization
#    if t%10 == 0:
#        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.pdf'%str(t/10))
#        #visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
#    t += 1
