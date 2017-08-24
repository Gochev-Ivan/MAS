import numpy as np
import time
from ConstantsAndParameters import *

def DrawFormation(number_of_drones, Rectangle):
    x1, x2, x3, x4 = Rectangle[0][0], Rectangle[1][0], Rectangle[2][0], Rectangle[3][0]
    y1, y2, y3, y4 = Rectangle[0][1], Rectangle[1][1], Rectangle[2][1], Rectangle[3][1]
    z1, z2, z3, z4 = Rectangle[0][2], Rectangle[1][2], Rectangle[2][2], Rectangle[3][2]
    """ Rectangle = [[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],[x4,y4,z4]] """
    # Draw Figure:
    ax = np.linspace(x1,x2,200)
    bx = np.linspace(x2,x3,200)
    cx = np.linspace(x3,x4,200)
    dx = np.linspace(x4,x1,200)
    
    ay = np.linspace(y1,y2,200)
    by = np.linspace(y2,y3,200)
    cy = np.linspace(y3,y4,200)
    dy = np.linspace(y4,y1,200)
    
    







if __name__ == "__main__":
    path = DrawFormation(number_of_drones, Rectangle)
    print Rectangle
    result = Rectangle