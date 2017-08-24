"""
COST FUNCTION FOR AUTO TUNING OF PID CONTROLLER
"""
def sum_lists(l1,l2):
    """ Arguments: list1, list2 """
    l = []
    for i in xrange(len(l1)):
        l.append(l1[i]+l2[i])
    return l
    
def dot_multiply(l, k):
    """ Arguments: list, scalar """
    l[:] = [x * k for x in l]
    return l
    
def cost(theta, sigma_err):
    """ theta = [Kp, Ki, Kd] """
    
    # step (step of sum TBD?):
    dt = 0.01
    
    """ sigma_err = sum of errors -> integral/sum function 
        -> These sums are gained in the code. """
    
    # Compute the integral:
    t0 = 0
    tf = 1
    
    J = (1/(tf - t0))*(sigma_err **2)*dt
    return J
    
def autotune(theta):
    
    # weight how much the parameters are changed each iteration:
    a = 0.1
    
    # Compute derivative with respec to first parameter:
    delta = 0.01
    var = [1, 0, 0]
    
    derivative = (cost(theta + dot_multiply(var,delta)) - cost(theta - dot_multiply(var,delta)))/(2 * delta)
    
    """ SIMULATION: theta = [Kp Ki Kd] => sim(theta) """    
    
    theta_new = theta - dot_multiply(derivative,a) #a*derivative
    print theta_new
    
    return theta_new
    
    def dot_multiply(l1, k):
        """ Arguments: list, scalar """
        ln = []
        for j in l1:
            pom = j*k
            ln.append(pom)
        return ln
        
if __name__ == "__main__":
    
    theta = [1,2,3]
    var = [1,0,0]
    delta = 0.01
    ls = []
    ls.append(1)
    print ls[0]
    
    
    
    
    
    
    
    
    
    
    
    
    
    