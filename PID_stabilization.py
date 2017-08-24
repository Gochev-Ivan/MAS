""" SCRIPT FOR CALLING CONTROLLER FOR A SPECIFIC QUADCOPTER """
def PIDc(targetPos, pos, l, sp, vx, vy, m, euler, orientation, cumul, last_e, pAlphaE, pBetaE, psp2, psp1, prevEuler, cumulAlpha, cumulBeta, cumulAlphaPos, cumulBetaPos, vParam, Kpv, Kiv, Kdv, Kph, Kih, Kdh, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, Kpr, Kir, Kdr, i):
    print targetPos, pos, l, sp, vx, vy, m, euler, orientation, cumul, last_e, pAlphaE, pBetaE, psp2, psp1, prevEuler, cumulAlpha, cumulBeta, cumulAlphaPos, cumulBetaPos, vParam, Kpv, Kiv, Kdv, Kph, Kih, Kdh, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, Kpr, Kir, Kdr, i
    import time
    import numpy as np
    from captains_log_v1 import log_data
    import os
    
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # output limits:
    #min_output=0
    #max_output=8.335
    # program parameters:
       
    e=targetPos[2]-pos[2]
    cumul=cumul+e
    diff_e=e-last_e        
    Pvert=Kpv*e
    Ivert=Kiv*cumul
    Dvert=Kdv*diff_e
    thrust=5.335+Pvert+Ivert+Dvert+l[2]*vParam# get thrust
    last_e=e
    
    
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
    
    ## WRITE TO TEXT:
#    log_data(C_parameters_vert, C_parameters_horr, C_parameters_rot, vert_comp, horr_comp, rot_comp, particlesTargetVelocities, i)
#    i +=1
    
    return particlesTargetVelocities
    
      
    
