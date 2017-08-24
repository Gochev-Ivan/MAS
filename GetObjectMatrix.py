def GetObjectMatrix(euler, pos):
    import math
    
    alpha = euler[0]
    beta = euler[1]
    gamma = euler[2]
    
    x = pos[0]
    y = pos[1]
    z = pos[2]

    M = []
    
    M.append([math.cos(beta)*math.cos(gamma),
                  -math.cos(beta)*math.sin(gamma),
                  math.sin(beta),x])
    
    M.append([math.cos(alpha)*math.sin(gamma)+math.cos(gamma)*math.sin(alpha)*math.sin(beta),\
              math.cos(alpha)*math.cos(gamma)-math.sin(alpha)*math.sin(beta)*math.sin(gamma),\
              -math.cos(beta)*math.sin(alpha),y])
    
    M.append([math.sin(alpha)*math.sin(gamma)-math.cos(alpha)*math.cos(gamma)*math.sin(beta),\
              math.cos(gamma)*math.sin(alpha)+math.cos(alpha)*math.sin(beta)*math.sin(gamma),\
              math.cos(alpha)*math.cos(beta),z])
    
    M.append([0,0,0,1])
    
    return M