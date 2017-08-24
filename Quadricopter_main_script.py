""" SCRIPT FOR CALLING CONTROLLER FOR A SPECIFIC QUADCOPTER """
#def QC_controller(Quadricopter_target, Quadricopter_base, Quadricopter):
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')
    
import time
import numpy as np
from captains_log_v1 import log_data
import os

import PID_stabilization

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

drone_targets = ['Quadricopter_target','Quadricopter_target#0','Quadricopter_target#1','Quadricopter_target#2']
bases = ['Quadricopter_base','Quadricopter_base#0','Quadricopter_base#1','Quadricopter_base#2']
fleet = ['Quadricopter','Quadricopter#0','Quadricopter#1','Quadricopter#2']

Quadricopter_target, Quadricopter_base, Quadricopter = drone_targets[0], bases[0], fleet[0]

""" Parameters: """
# output limits:
#min_output=0
#max_output=8.335
# program parameters:
global i
i = 0
xe = []
ye = []
ze = []
xs = []
ys = []
zs = []
x_qc = []
y_qc = []
z_qc = []
u = []
v1 = []
v2 = []
v3 = []
v4 = []
#global variables:
cumul=0
last_e=0
pAlphaE=0
pBetaE=0
psp2=0
psp1=0
prevEuler=0

cumulAlpha = 0
cumulBeta = 0

cumulAlphaPos = 0
cumulBetaPos = 0

particlesTargetVelocities=[0,0,0,0]

# weight how much the parameters are changed each iteration:
a = 0.1

# Compute derivative with respec to first parameter:
delta = 0.01

# control signals parameters:
last_thrust = 0

#speed weight:
vParam=-2
#parameters for vertical control
Kpv=2
Kiv=0
Kdv=2
#parameters for horizontal control:
Kph=0.4
Kih=0.1
Kdh=1.5
Kph_pos1=0.4
Kih_pos1=0.001
Kdh_pos1=0.05
Kph_pos0=0.4
Kih_pos0=0.001
Kdh_pos0=0.05
#parameters for rotational control:
Kpr=0.05
Kir=0
Kdr=0.9
""" ===================================================================="""

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')

    # enable the synchronous mode on the client:
    vrep.simxSynchronous(clientID,True)

    # start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)

    #functional/handle code:
    emptyBuff=bytearray()
    [returnCode,targetObj]=vrep.simxGetObjectHandle(clientID,Quadricopter_target,vrep.simx_opmode_blocking)
    [returnCode,qc_base_handle]=vrep.simxGetObjectHandle(clientID,Quadricopter_base,vrep.simx_opmode_blocking)
    [returnCode,qc_handle]=vrep.simxGetObjectHandle(clientID,Quadricopter,vrep.simx_opmode_blocking)
    
    # main loop:
    while True:
        
        '''==========''' ''' Data for vertical control: ''' '''=========='''
        [returnCode,targetPos]=vrep.simxGetObjectPosition(clientID,targetObj,-1,vrep.simx_opmode_blocking)
        [returnCode,pos]=vrep.simxGetObjectPosition(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
        [returnCode, l, w] = vrep.simxGetObjectVelocity(clientID, qc_base_handle, vrep.simx_opmode_blocking)
        
        '''==========''' ''' Data for horizontal control: ''' '''=========='''
        [returnCode,sp]=vrep.simxGetObjectPosition(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
        [rc,rc,vx,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_vx',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
        [rc,rc,vy,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_vy',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
        [rc,rc,rc,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_Object_Matrix',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
        [errorCode,M]=vrep.simxGetStringSignal(clientID,'mtable',vrep.simx_opmode_oneshot_wait);
        if (errorCode==vrep.simx_return_ok):
            m=vrep.simxUnpackFloats(M)
        
        '''==========''' ''' Data for rotational control: ''' '''=========='''
        [returnCode,euler]=vrep.simxGetObjectOrientation(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
        
        # send propeller velocities to output:
        particlesTargetVelocities = PID_stabilization.PIDc(targetPos, pos, l, sp, vx, vy, m, euler, orientation, cumul, last_e, pAlphaE, pBetaE, psp2, psp1, prevEuler, cumulAlpha, cumulBeta, cumulAlphaPos, cumulBetaPos, vParam, Kpv, Kiv, Kdv, Kph, Kih, Kdh, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, Kpr, Kir, Kdr, i)
        
        [res,retInts,retFloats,retStrings,retBuffer]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_propeller_v',[],particlesTargetVelocities,[],emptyBuff,vrep.simx_opmode_blocking)    
        vrep.simxSynchronousTrigger(clientID)
        
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
    
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
        
else:
    print ('Failed connecting to remote API server')      