import random

""" Formations: """
number_of_drones = 4
#x1 = 0
#x2 = 1
#y1 = 0
#y2 = 1
#z1 = z2 = 1
#x = [x1, x2, x2, x1, x1]
x1, x2, x3, x4 = random.randint(0,3), random.randint(0,3), random.randint(0,3), random.randint(0,3)
y1, y2, y3, y4 = random.randint(0,3), random.randint(0,3), random.randint(0,3), random.randint(0,3)
z1, z2, z3, z4 = 1, 1, 1, 1
#Rectangle = [[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],[x4,y4,z4]] # figure = [[x],[y],[z]]
Rectangle = [[1,1,1],[-1,1,1],[-1,-1,1],[1,-1,1],[1,1,1]]