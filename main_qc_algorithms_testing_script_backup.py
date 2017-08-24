""" SCRIPT FOR CALLING CONTROLLER FOR A SPECIFIC QUADCOPTER """
def QC_controller(Quadricopter_target, Quadricopter_base, Quadricopter):
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
    """ =========================================================== """
    """ ============ Imported Libraries: ============ """    
    import time
    import numpy as np
    from captains_log_v1 import log_data
    import os
    import cost
    import csv
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    """ =================================================================== """
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
    
    s_r = 0
    
    particlesTargetVelocities=[0,0,0,0]
    #speed weight:
    vParam=-2
    #parameters for vertical control
    Kpv=2
    Kiv=0
    Kdv=2
    #parameters for horizontal control:
    Kph=0.4 # TBD
    Kih=0.1 # TBD
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
    """ =========================================================== """
    # parameters needed for gradient descent:
    t0 = 0
    tf = 1
    dt = 0.01
    
    sum_h_alpha = []
    sum_h_beta = []
    sum_h_pos0 = []
    sum_h_pos1 = []
    
    last_alpha_angle = 0
    last_alpha_pos = 0
    last_beta_angle = 0
    last_beta_pos = 0
       
    delta_alpha_angle = []
    delta_alpha_pos = []
    delta_beta_angle = []
    delta_beta_pos = []
    
    J_h_alpha = []
    J_h_beta = []
    J_h_pos0 = []
    J_h_pos1 = []
    
    paramX = []
    paramY = []
    
    theta_h_angles = [Kph, Kih, Kdh]
    theta_h_pos = [Kph_pos0, Kih_pos0, Kdh_pos0] # Kph_pos0 == Kph_pos1 ...
    
    gd = 0
    """ =========================================================== """
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
            """ ========== vertical control: ========== """
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

            """ =========================================================== """
            """ ========== horizontal control: ========== """
            [returnCode,sp]=vrep.simxGetObjectPosition(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
            [rc,rc,vx,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_vx',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
            [rc,rc,vy,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_vy',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
            [rc,rc,rc,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_Object_Matrix',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
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
            
            delta_alpha_angle.append(alphaCorr - last_alpha_angle)
            delta_beta_angle.append(betaCorr - last_beta_angle)
            delta_alpha_pos.append(alphaPos - last_alpha_pos)
            delta_beta_pos.append(betaPos - last_beta_pos)
            last_alpha_angle = alphaCorr
            last_beta_angle = betaCorr
            last_alpha_pos = alphaPos
            last_beta_pos = betaPos
            
            """ =========================================================== """            
            """ ========== rotational control: ========== """
            [returnCode,euler]=vrep.simxGetObjectOrientation(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
            [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
            
            Prot=Kpr*euler[2]
            Drot=Kdr*(euler[2]-prevEuler)
            s_r = s_r + euler[2]
            rotCorr=Prot+Drot
            prevEuler=euler[2]
            
            """ =========================================================== """
            """ ========== set propeller velocities: ========== """
            propeller1_PTV = thrust*(1-alphaCorr+betaCorr-rotCorr)
            propeller2_PTV = thrust*(1-alphaCorr-betaCorr+rotCorr)
            propeller3_PTV = thrust*(1+alphaCorr-betaCorr-rotCorr)
            propeller4_PTV = thrust*(1+alphaCorr+betaCorr+rotCorr)
            particlesTargetVelocities=[propeller1_PTV, propeller2_PTV, propeller3_PTV, propeller4_PTV]
            
            """ =========================================================== """
            """ ========== set parameters for logging data: ========== """
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
            
            """ =========================================================== """
            """ ========== PLOTTING: ========== """
            ## PLOTTING:
            ze.append(e) # otstapuvanje od z-oska
            xe.append(sp[0]) # otstapuvanje od x-oska
            ye.append(sp[1]) # otstapuvawe od y-oska
#            xs.append(targetPos[0])
#            ys.append(targetPos[1])
#            zs.append(targetPos[2])
#            x_qc.append(pos[0])
#            y_qc.append(pos[1])
#            z_qc.append(pos[2])
#            fig = plt.figure()
#            ax = fig.add_subplot(111, projection='3d')
#            ax.plot_wireframe(xs,ys,zs,color='blue')
#            ax.plot_wireframe(x_qc,y_qc,z_qc,color='red')
#            ax.set_xlim3d(-2.1,2.1)
#            ax.set_ylim3d(-2.1,2.1)
#            ax.set_zlim3d(0,1.9)
#            plt.show()
            
#            u.append(thrust)
#            v1.append(propeller1_PTV)
#            v2.append(propeller2_PTV)
#            v3.append(propeller3_PTV)
#            v4.append(propeller4_PTV)
#            
            plt.plot(xe,color='blue')
            plt.hold(True)
            plt.plot(ye,color='red')
            plt.hold(True)
            plt.plot(ze,color='green')
            plt.hold(True)
#            plt.plot(v4,color='pink')
#            plt.hold(True)
#            plt.axis([0,25,0,15])
#            plt.show()
#            
#            plt.plot(xe,color='blue')
#            plt.axis([0,100,-4,4])
#            plt.show()
#            plt.plot(ye,color='red')
#            plt.axis([0,100,-4,4])
#            plt.show()
#            plt.plot(ze,color='green')
            plt.axis([0,25,-2,2])
            plt.show()
            
            """ =========================================================== """
            """ ========== Gradient descent: ========== """
            sum_h_alpha.append(cumulAlpha)
            sum_h_beta.append(cumulBeta)
            sum_h_pos1.append(cumulAlphaPos)
            sum_h_pos0.append(cumulBetaPos)
            
            J_h_alpha.append((1/(tf - t0))*(5*cumulAlpha**2 + delta_alpha_angle[gd]**2)*dt)
            J_h_beta.append((1/(tf - t0))*(5*cumulBeta**2 + delta_beta_angle[gd]**2)*dt)
            J_h_pos1.append((1/(tf - t0))*(5*cumulAlphaPos**2 + delta_alpha_pos[gd]**2)*dt)
            J_h_pos0.append((1/(tf - t0))*(5*cumulBetaPos**2 + delta_beta_pos[gd]**2)*dt)
            
            paramX.append(targetPos[0])
            paramY.append(targetPos[1])
            
            print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
            print "theta_h_angles : ", theta_h_angles
            print "theta_h_pos : ", theta_h_pos
            
            print "sum_h_alpha : ", sum_h_alpha[gd]
            print "sum_h_beta : ", sum_h_beta[gd]
            print "sum_h_pos1 : ", sum_h_pos1[gd]
            print "sum_h_pos0 : ", sum_h_pos0[gd]
            
            print "J_h_alpha : ", J_h_alpha[gd]
            print "J_h_beta : ", J_h_beta[gd]
            print "J_h_pos1 : ", J_h_pos1[gd]
            print "J_h_pos0 : ", J_h_pos0[gd]
            
            print "paramX : ", paramX[gd]
            print "paramY : ", paramY[gd]
            print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
            gd = gd + 1
            print "gd (iterations) : ", gd
            print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
            
            """ =========================================================== """
            if gd == 400:
                with open('GD_43.csv','wb') as f1:
                    writer=csv.writer(f1)
                    for i in range(0, gd):
                        writer.writerow([paramX[i],paramY[i],theta_h_angles[0],theta_h_angles[1],theta_h_angles[2],theta_h_pos[0],theta_h_pos[1],theta_h_pos[2],J_h_alpha[i],J_h_beta[i],J_h_pos1[i],J_h_pos0[i]])
                break
            
            """ =========================================================== """
            """ ========== WRITE TO TEXT: ========== """
            ## WRITE TO TEXT:
            log_data(C_parameters_vert, C_parameters_horr, C_parameters_rot, vert_comp, horr_comp, rot_comp, particlesTargetVelocities, i)
            i +=1
            """ =========================================================== """
           # send propeller velocities to output:
            [res,retInts,retFloats,retStrings,retBuffer]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_propeller_v',[],particlesTargetVelocities,[],emptyBuff,vrep.simx_opmode_blocking)    
            vrep.simxSynchronousTrigger(clientID)
            """ =========================================================== """
        # stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
        
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
            
    else:
        print ('Failed connecting to remote API server')
        