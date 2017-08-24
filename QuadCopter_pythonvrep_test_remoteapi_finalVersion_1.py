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

# output limits:
#min_output=0
#max_output=8.335
# program parameters:
global i
i = 0
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
    [returnCode,targetObj]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_blocking)
    [returnCode,qc_base_handle]=vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_blocking)
    [returnCode,qc_handle]=vrep.simxGetObjectHandle(clientID,'Quadricopter',vrep.simx_opmode_blocking)
    
    # main loop:
    while True:
        # vertical control:
        [returnCode,targetPos]=vrep.simxGetObjectPosition(clientID,targetObj,-1,vrep.simx_opmode_blocking)
        [returnCode,pos]=vrep.simxGetObjectPosition(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
        [returnCode, l, w] = vrep.simxGetObjectVelocity(clientID, qc_base_handle, vrep.simx_opmode_blocking)
        
        e=targetPos[2]-pos[2]
        cumul=cumul+e
        diff_e=e-last_e        
        Pvert=Kpv*e
        Ivert=Kiv*cumul
        Dvert=Kdv*diff_e
        thrust=5.335+Pvert+Ivert+Dvert+l[2]*vParam# get thrust
        last_e=e
        # horizontal control:
        [returnCode,sp]=vrep.simxGetObjectPosition(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
        [rc,rc,vx,rc,rc]=vrep.simxCallScriptFunction(clientID,'Quadricopter',vrep.sim_scripttype_childscript,'qc_Get_vx',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
        [rc,rc,vy,rc,rc]=vrep.simxCallScriptFunction(clientID,'Quadricopter',vrep.sim_scripttype_childscript,'qc_Get_vy',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
        [rc,rc,rc,rc,rc]=vrep.simxCallScriptFunction(clientID,'Quadricopter',vrep.sim_scripttype_childscript,'qc_Get_Object_Matrix',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
        [errorCode,M]=vrep.simxGetStringSignal(clientID,'mtable',vrep.simx_opmode_oneshot_wait);
        if (errorCode==vrep.simx_return_ok):
            m=vrep.simxUnpackFloats(M)
        
        alphaE=vy[2]-m[11]
        cumulAlpha = cumulAlpha + alphaE
        diff_alphaE=alphaE-pAlphaE
        alphaCorr=Kph*alphaE + Kih*cumulAlpha + Kdh*diff_alphaE #alpha correction
        
        betaE=vx[2]-m[11]
        cumulBeta = cumulBeta + betaE
        diff_betaE=betaE-pBetaE
        betaCorr=-Kph*betaE - Kih*cumulBeta - Kdh*diff_betaE #beta correction
        
        
        pAlphaE=alphaE
        pBetaE=betaE
        
        cumulAlphaPos = cumulAlphaPos + sp[1]
        cumulBetaPos = cumulBetaPos + sp[0]
        alphaPos=Kph_pos1*(sp[1])+ Kih_pos1*cumulAlphaPos +Kdh_pos1*(sp[1]-psp2) #alpha position correction
        betaPos=Kph_pos0*(sp[0])+ Kih_pos0*cumulBetaPos + Kdh_pos0*(sp[0]-psp1) #beta position correction
        
        alphaCorr=alphaCorr+alphaPos
        betaCorr=betaCorr-betaPos
        
        psp2=sp[1]
        psp1=sp[0]
        # rotational control:
        [returnCode,euler]=vrep.simxGetObjectOrientation(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
        
        Prot=Kpr*euler[2]
        Drot=Kdr*(euler[2]-prevEuler)
        rotCorr=Prot+Drot
        prevEuler=euler[2]
        
        # set propeller velocities:
        propeller1_PTV = thrust*(1-alphaCorr+betaCorr-rotCorr)
        propeller2_PTV = thrust*(1-alphaCorr-betaCorr+rotCorr)
        propeller3_PTV = thrust*(1+alphaCorr-betaCorr-rotCorr)
        propeller4_PTV = thrust*(1+alphaCorr+betaCorr+rotCorr)
        particlesTargetVelocities=[propeller1_PTV, propeller2_PTV, propeller3_PTV, propeller4_PTV]
        
        C_parameters_vert = [0,0,0]
        C_parameters_horr = [0,0,0,0,0,0,0]
        C_parameters_rot = [0,0,0]
        vert_comp = [0,0,0,0]
        horr_comp = [0,0,0,0,[0,0,0],0,0,0,0,0,0]
        rot_comp = [[0,0,0],0]
        
        C_parameters_vert[0] = Kpv
        C_parameters_vert[1] = Kiv
        C_parameters_vert[2] = Kdv
        
        C_parameters_horr[0] = Kph
        C_parameters_horr[1] = Kih
        C_parameters_horr[2] = Kdh
        C_parameters_horr[3] = Kph_pos0
        C_parameters_horr[4] = Kdh_pos0
        C_parameters_horr[5] = Kph_pos1
        C_parameters_horr[6] = Kdh_pos1
        
        C_parameters_rot[0] = Kpr
        C_parameters_rot[1] = Kir
        C_parameters_rot[2] = Kdr
        
        vert_comp[0] = targetPos
        vert_comp[1] = pos
        vert_comp[2] = e
        vert_comp[3] = thrust
        
        horr_comp[0] = alphaE
        horr_comp[1] = betaE
        horr_comp[2] = cumulAlpha
        horr_comp[3] = cumulBeta
        horr_comp[4] = sp
        horr_comp[5] = cumulAlphaPos
        horr_comp[6] = cumulBetaPos
        horr_comp[7] = alphaCorr
        horr_comp[8] = betaCorr
        horr_comp[9] = vx
        horr_comp[10] = vy
        
        rot_comp[0] = euler
        rot_comp[1] = rotCorr
        
#        print '////////////'
#        print '------------'
#        print '-Controller parameters-'
#        print 'Vertical control parameters=',[Kpv,Kiv,Kdv]
#        print 'Horizontal control parameters=',[Kph,Kih,Kdh,Kph_pos0,Kdh_pos0,Kph_pos1,Kdh_pos1]
#        print 'Rotational control parameters=',[Kpr,Kir,Kdr]
#        print '------------'
#        print '-Vertical component-'
#        print 'targetPos=',targetPos
#        print 'pos=',pos        
#        print 'e=',e
#        print 'thrust=',thrust        
#        print '------------'
#        print '-Horizontal component-'
#        print 'cumulAlpha=',cumulAlpha
#        print 'cumulBeta=',cumulBeta
#        print 'cumulAlphaPos=',cumulAlphaPos
#        print 'cumulBetaPos=',cumulBetaPos
#        print 'alphaE=',alphaE
#        print 'betaE=',betaE
#        print 'alphaCorr=',alphaCorr
#        print 'betaCorr=',betaCorr
#        print 'orientation=',orientation
#        print 'sp_X=',sp[0]
#        print 'sp_Y=',sp[1]
#        print '------------'
#        print '-Rotational component-'
#        print 'vx=',vx
#        print 'vy=',vy
#        print 'gamma=',euler[2]
#        print 'rotCorr=',rotCorr
#        print '------------'
#        print 'Velocity_propeller_1 = ',particlesTargetVelocities[0]
#        print 'Velocity_propeller_2 = ',particlesTargetVelocities[1]
#        print 'Velocity_propeller_3 = ',particlesTargetVelocities[2]
#        print 'Velocity_propeller_4 = ',particlesTargetVelocities[3]      
#        print '------------'
#        print '////////////'
        
        ## WRITE TO TEXT:
        log_data(C_parameters_vert, C_parameters_horr, C_parameters_rot, vert_comp, horr_comp, rot_comp, particlesTargetVelocities, i)
        i +=1
       # send propeller velocities to output:
        [res,retInts,retFloats,retStrings,retBuffer]=vrep.simxCallScriptFunction(clientID,'Quadricopter',vrep.sim_scripttype_childscript,'qc_propeller_v',[],particlesTargetVelocities,[],emptyBuff,vrep.simx_opmode_blocking)    
        vrep.simxSynchronousTrigger(clientID)
        
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
    
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
        
else:
    print ('Failed connecting to remote API server')      