import numpy as np
import scipy.interpolate
import matplotlib.pyplot as plt

def interpolate_data(sequence):
    # sequence = result from A* pathfinder
    x = [row[0] for row in sequence]
    y = [row[1] for row in sequence]
    z = [row[2] for row in sequence]
      
    
#    interp = scipy.interpolate.Rbf(x, y, z, function='cubic')
    print x
    print y
    print z
#    print interp
    
#    fig = plt.figure()
#    ax = fig.add_subplot(111, projection='3d')
#    ax.plot_wireframe(xi, yi, zi)
#    plt.show()
    
# main:
#sequence = [[1,2,3],[4,5,6],[7,8,9]]
#interpolate_data(sequence)