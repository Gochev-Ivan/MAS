#import vrep
import math
import collections
import Queue
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#from interpolate_points import interpolate_data
import scipy.interpolate
#from pythonvrep_test_remoteapi_v7 import quadcopter_fcn
import numpy as np
import pandas as pnd
import csv

class Quadcopter:

    def __init__(self, clientID,spawn_point, end_point):
        self.clientID = clientID
        self.end_point = end_point
        self.spawn_point = spawn_point
        # self.modelPath = './Models/Quadricopter.ttm'
        # return_code, self.quad_handle = vrep.simxLoadModel(self.clientID, self.modelPath, 1, vrep.simx_opmode_blocking)
        # vrep.simxSetObjectPosition(self.clientID,self.quad_handle,-1,spawn_point,vrep.simx_opmode_oneshot)


    def move_to_point(self,point):
        #do implementation of PID
        return;

    def get_neighbors(self,point):
        return [[point[0] + 1,point[1],point[2]],
                [point[0] - 1, point[1], point[2]],
                [point[0], point[1] + 1, point[2]],
                [point[0], point[1] - 1, point[2]],
                [point[0], point[1], point[2] + 1],
                [point[0], point[1], point[2] - 1],
                [point[0] + 1, point[1] + 1, point[2]],
                [point[0] + 1, point[1] - 1, point[2]],
                [point[0] - 1, point[1] + 1, point[2]],
                [point[0] - 1, point[1] - 1, point[2]],
                [point[0], point[1] + 1, point[2] + 1],
                [point[0], point[1] + 1, point[2] -1],
                [point[0], point[1] - 1, point[2] + 1],
                [point[0], point[1] - 1, point[2] - 1],
                [point[0]-1, point[1], point[2] + 1],
                [point[0]-1, point[1], point[2] - 1],
                [point[0]+1, point[1], point[2] + 1],
                [point[0]+1, point[1], point[2] - 1],
                [point[0] + 1, point[1] + 1, point[2] + 1],
                [point[0] + 1, point[1] + 1, point[2] - 1],
                [point[0] + 1, point[1] - 1, point[2] + 1],
                [point[0] + 1, point[1] - 1, point[2] - 1],
                [point[0] - 1, point[1] + 1, point[2] + 1],
                [point[0] - 1, point[1] + 1, point[2] - 1],
                [point[0] - 1, point[1] - 1, point[2] + 1],
                [point[0] - 1, point[1] - 1, point[2] - 1]]


    def d_star(self):
        list_of_points = []

        #todo: D* algo impl

        return list_of_points

    def step_cost(self, point, next_point):
        if point[2] == next_point[2]:
            if (point[1] != next_point[1] and point[0] == next_point[0]) or (
                    point[1] == next_point[1] and point[0] != next_point[0]):
                return 10
            elif point[1] != next_point[1] and point[0] != next_point[0]:
                return 14.44

        if point[2] != next_point[2]:
            if point[1] == next_point[1] and point[0] == next_point[0]:
                return 10
            if point[1] != next_point[1] and point[0] != next_point[0]:
                return 17.5
            return 30
        return 0

    def a_star(self):
        list_of_points = []
        # front = [vrep.simxGetObjectPosition(self.clientID, self.quad_handle, -1,vrep.simx_opmode_oneshot)]
        front = [self.spawn_point]
        came_from = {}
        cost_so_far = {}
        closedSet = set()
        came_from[str(front[0])] = None
        cost_so_far[str(front[0])] = 0
        i = 0;
        closedSet.add(str(front[0]))

        while len(front):

            current_position = front.pop(0)
            if current_position == self.end_point:
                break

            for neighbour in self.get_neighbors(current_position):

                new_cost = cost_so_far[str(current_position)] + self.step_cost(current_position, neighbour)
                if str(neighbour) not in cost_so_far or new_cost < cost_so_far[str(neighbour)]:
                    priority = self.heuristics(neighbour)
                    front.insert(int(priority), neighbour)
                    came_from[str(neighbour)] = current_position
                    cost_so_far[str(neighbour)] = new_cost
                    closedSet.add(str(neighbour))


        pos = self.end_point
        list_of_points = [pos]

        while str(pos) in came_from.keys():
            list_of_points.append(came_from[str(pos)])
            pos = came_from[str(pos)]

        list_of_points.reverse()

        return list_of_points[1:]

    def heuristics(self, point):
        return math.sqrt((point[0] - self.end_point[0])**2 +(point[1] - self.end_point[1])**2 + (point[2] - self.end_point[2])**2)

quad = Quadcopter(1, [0,0,0],[10,10,3])
timer = time.time()
result = quad.a_star()
print time.time() - timer
print result
print len(result)
xs = [x[0] for x in result]
ys = [x[1] for x in result]
zs = [x[2] for x in result]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_wireframe(xs,ys,zs)
plt.show()
# ----------------------------------------------------------------------------|
# INTERPOLATION:
#print len(xs)
#print len(ys)
#print len(zs)
#interpolate_data(result)

# TRAJECTORY TRACKING:
#quadcopter_fcn(result)

# WRITE IN CSV FILE:
# %%
for i in range(0,len(result)):
    for j in range(0,3):
        result[i][j] = str(result[i][j])
print result
# %%
with open('test_list_Astar_8.csv','w') as f1:
    writer=csv.writer(f1)
    for i in range(0, len(result)):
        first = ','.join(result[i])
        form = '%s' % first
        print form
        writer.writerow(form)
# %% OVA RABOTI!!!
with open('test_list_Astar_15.csv','wb') as f1:
    writer=csv.writer(f1)
    for i in range(0, len(result)):
        writer.writerow([result[i][0],result[i][1],result[i][2]])
# %%
for i in range(0, len(result)):
    f1 = open('test_list_Astar_10.csv','wb')
    first = ','.join(result[i])
    form = '%s' % first
    print form
    f1.write(form)
    f1.close()
# %%
a = ['a','b','c']
first = '", "'.join(a)
second = '"%s"' % first
print second
