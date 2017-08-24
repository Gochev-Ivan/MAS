""" SCRIPT FOR CALLING CONTROLLER FOR A SPECIFIC QUADCOPTER """
def QC_controller(Quadricopter_target, Quadricopter_base, Quadricopter, Quadricopter_target2, Quadricopter_base2, Quadricopter2, Quadricopter_target0, Quadricopter_base0, Quadricopter0):
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
    import main_PID_controller
    """ =================================================================== """
    # output limits:
    #min_output=0
    #max_output=8.335
    # program parameters:
    global i
    i = 0
    xf2 = []
    xf0 = []
    xl = []
    yl = []
    yf2 = []
    yf0 = []
    zl = []
    zf2 = []
    zf0 = []
    vxl = []
    vxf = []
    vyl = []
    vyf = []
    euler_l = []
#    euler_f2 = []
#    euler_f0 = []
#    xe = []
#    ye = []
    ze2 = []
    ze0 = []
    xe2 = []
    xe0 = []
    ye2 = []
    ye0 = []
#    xs = []
#    ys = []
#    zs = []
    x_qc = []
    y_qc = []
    z_qc = []
    x_qc2 = []
    y_qc2 = []
    z_qc2 = []
#    u = []
    v1 = []
    v2 = []
    v3 = []
    v4 = []
    v12 = []
    v22 = []
    v32 = []
    v42 = []
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
    
    
    cumul2=0
    last_e2=0
    pAlphaE2=0
    pBetaE2=0
    psp22=0
    psp12=0
    prevEuler2=0
    
    cumulAlpha2 = 0
    cumulBeta2 = 0
    
    cumulAlphaPos2 = 0
    cumulBetaPos2 = 0
    
    
    cumul0=0
    last_e0=0
    pAlphaE0=0
    pBetaE0=0
    psp20=0
    psp10=0
    prevEuler0=0
    
    cumulAlpha0 = 0
    cumulBeta0 = 0
    
    cumulAlphaPos0 = 0
    cumulBetaPos0 = 0
    
    
    cumulPOS = 0
    cumulVY = 0
    cumulVX = 0
    cumulSP0 = 0
    cumulSP1 = 0
    cumulEULER = 0
    cumulPOS2 = 0
    cumulVY2 = 0
    cumulVX2 = 0
    cumulSP02 = 0
    cumulSP12 = 0
    cumulEULER2 = 0
    cumulPOS0 = 0
    cumulVY0 = 0
    cumulVX0 = 0
    cumulSP00 = 0
    cumulSP10 = 0
    cumulEULER0 = 0
    
    particlesTargetVelocities=[0,0,0,0]
    particlesTargetVelocities2=[0,0,0,0]
    particlesTargetVelocities0=[0,0,0,0]
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
#    Kir=0
    Kdr=0.9
    
    """ =========================================================== """    
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
        
        [returnCode,targetObj2]=vrep.simxGetObjectHandle(clientID,Quadricopter_target2,vrep.simx_opmode_blocking)
        [returnCode,qc_base_handle2]=vrep.simxGetObjectHandle(clientID,Quadricopter_base2,vrep.simx_opmode_blocking)
        [returnCode,qc_handle2]=vrep.simxGetObjectHandle(clientID,Quadricopter2,vrep.simx_opmode_blocking)
        
        [returnCode,targetObj0]=vrep.simxGetObjectHandle(clientID,Quadricopter_target0,vrep.simx_opmode_blocking)
        [returnCode,qc_base_handle0]=vrep.simxGetObjectHandle(clientID,Quadricopter_base0,vrep.simx_opmode_blocking)
        [returnCode,qc_handle0]=vrep.simxGetObjectHandle(clientID,Quadricopter0,vrep.simx_opmode_blocking)
        # main loop:
        while True:
            """ ========== vertical control: ========== """
            [returnCode,targetPos]=vrep.simxGetObjectPosition(clientID,targetObj,-1,vrep.simx_opmode_blocking)
            [returnCode,pos]=vrep.simxGetObjectPosition(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
            [returnCode, l, w] = vrep.simxGetObjectVelocity(clientID, qc_base_handle, vrep.simx_opmode_blocking)
            
            [returnCode,targetPos2]=vrep.simxGetObjectPosition(clientID,targetObj2,-1,vrep.simx_opmode_blocking)
            [returnCode,pos2]=vrep.simxGetObjectPosition(clientID,qc_base_handle2,-1,vrep.simx_opmode_blocking)
            [returnCode, l2, w2] = vrep.simxGetObjectVelocity(clientID, qc_base_handle2, vrep.simx_opmode_blocking)
            
            [returnCode,targetPos0]=vrep.simxGetObjectPosition(clientID,targetObj0,-1,vrep.simx_opmode_blocking)
            [returnCode,pos0]=vrep.simxGetObjectPosition(clientID,qc_base_handle0,-1,vrep.simx_opmode_blocking)
            [returnCode, l0, w0] = vrep.simxGetObjectVelocity(clientID, qc_base_handle0, vrep.simx_opmode_blocking)
            
            """ =========================================================== """
            """ ========== horizontal control: ========== """
            [returnCode,sp]=vrep.simxGetObjectPosition(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
            [rc,rc,vx,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_vx',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
            [rc,rc,vy,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_vy',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
            [rc,rc,rc,rc,rc]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_Get_Object_Matrix',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
            [errorCode,M]=vrep.simxGetStringSignal(clientID,'mtable',vrep.simx_opmode_oneshot_wait);
            if (errorCode==vrep.simx_return_ok):
                m=vrep.simxUnpackFloats(M)

            m1 = m[0]
            m2 = m[1]
            m0 = m[2]
            vx1 = [vx[0],vx[1],vx[2]]
            vx2 = [vx[3],vx[4],vx[5]]
            vx0 = [vx[6],vx[7],vx[8]]
            vy1 = [vy[0],vy[1],vy[2]]
            vy2 = [vy[3],vy[4],vy[5]]
            vy0 = [vy[6],vy[7],vy[8]]
            
            [returnCode,sp2]=vrep.simxGetObjectPosition(clientID,targetObj2,qc_base_handle2,vrep.simx_opmode_blocking)
            
            [returnCode,sp0]=vrep.simxGetObjectPosition(clientID,targetObj0,qc_base_handle0,vrep.simx_opmode_blocking)
            
            """ =========================================================== """            
            """ ========== rotational control: ========== """
            [returnCode,euler]=vrep.simxGetObjectOrientation(clientID,targetObj,qc_base_handle,vrep.simx_opmode_blocking)
            [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,qc_base_handle,-1,vrep.simx_opmode_blocking)
            
            [returnCode,euler2]=vrep.simxGetObjectOrientation(clientID,targetObj2,qc_base_handle2,vrep.simx_opmode_blocking)
            [returnCode,orientation2]=vrep.simxGetObjectOrientation(clientID,qc_base_handle2,-1,vrep.simx_opmode_blocking)
            
            [returnCode,euler0]=vrep.simxGetObjectOrientation(clientID,targetObj0,qc_base_handle0,vrep.simx_opmode_blocking)
            [returnCode,orientation0]=vrep.simxGetObjectOrientation(clientID,qc_base_handle0,-1,vrep.simx_opmode_blocking)
            
            """ =========================================================== """
            """ ========== set propeller velocities: ========== """
            particlesTargetVelocities, cumul, last_e, cumulAlpha, pAlphaE, cumulBeta, pBetaE, cumulAlphaPos, cumulBetaPos, psp2, psp1, prevEuler, cumulPOS, cumulVY, cumulVX, cumulSP0, cumulSP1, cumulEULER = main_PID_controller.PID(targetPos, pos, cumul, last_e, Kpv, Kiv, Kdv, vParam, l, vy1, m1, cumulAlpha, pAlphaE, Kph, Kih, Kdh, vx1, cumulBeta, pBetaE, cumulAlphaPos, cumulBetaPos, sp, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, psp2, psp1, Kpr, Kdr, euler, prevEuler, cumulPOS, cumulVY, cumulVX, cumulSP0, cumulSP1, cumulEULER, vx, vy, pos, sp, euler) 
            particlesTargetVelocities2, cumul2, last_e2, cumulAlpha2, pAlphaE2, cumulBeta2, pBetaE2, cumulAlphaPos2, cumulBetaPos2, psp22, psp12, prevEuler2, cumulPOS2, cumulVY2, cumulVX2, cumulSP02, cumulSP12, cumulEULER2 = main_PID_controller.PID(targetPos2, pos2, cumul2, last_e2, Kpv, Kiv, Kdv, vParam, l2, vy2, m2, cumulAlpha2, pAlphaE2, Kph, Kih, Kdh, vx2, cumulBeta2, pBetaE2, cumulAlphaPos2, cumulBetaPos2, sp2, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, psp22, psp12, Kpr, Kdr, euler2, prevEuler2, cumulPOS2, cumulVY2, cumulVX2, cumulSP02, cumulSP12, cumulEULER2,vx, vy, pos, sp, euler)
            particlesTargetVelocities0, cumul0, last_e0, cumulAlpha0, pAlphaE0, cumulBeta0, pBetaE0, cumulAlphaPos0, cumulBetaPos0, psp20, psp10, prevEuler0, cumulPOS0, cumulVY0, cumulVX0, cumulSP00, cumulSP10, cumulEULER0 = main_PID_controller.PID(targetPos0, pos0, cumul0, last_e0, Kpv, Kiv, Kdv, vParam, l0, vy0, m0, cumulAlpha0, pAlphaE0, Kph, Kih, Kdh, vx0, cumulBeta0, pBetaE0, cumulAlphaPos0, cumulBetaPos0, sp0, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, psp20, psp10, Kpr, Kdr, euler0, prevEuler0, cumulPOS0, cumulVY0, cumulVX0, cumulSP00, cumulSP10, cumulEULER0,vx, vy, pos, sp, euler)
            
            particlesTargetVelocities_s = [particlesTargetVelocities[0],particlesTargetVelocities[1],particlesTargetVelocities[2],particlesTargetVelocities[3],particlesTargetVelocities2[0],particlesTargetVelocities2[1],particlesTargetVelocities2[2],particlesTargetVelocities2[3], particlesTargetVelocities0[0], particlesTargetVelocities0[1], particlesTargetVelocities0[2], particlesTargetVelocities0[3]]
            """ =========================================================== """
            """ =========================================================== """
            """ ========== PLOTTING: ========== """
            ## PLOTTING:
#            ze.append(targetPos[2]-pos[2]) # otstapuvanje od z-oska
#            xe.append(sp[0]) # otstapuvanje od x-oska
#            ye.append(sp[1]) # otstapuvawe od y-oska
#            xs.append(targetPos[0])
#            ys.append(targetPos[1])
#            zs.append(targetPos[2])
#            x_qc.append(pos[0])
#            y_qc.append(pos[1])
#            z_qc.append(pos[2])
#            x_qc2.append(pos2[0])
#            y_qc2.append(pos2[1])
#            z_qc2.append(pos2[2])
#            fig = plt.figure()
#            ax = fig.add_subplot(111, projection='3d')
#            ax.plot_wireframe(x_qc,y_qc,z_qc,color='blue')
#            ax.plot_wireframe(x_qc2,y_qc2,z_qc2,color='red')
#            ax.set_xlim3d(-2.1,2.1)
#            ax.set_ylim3d(-2.1,2.1)
#            ax.set_zlim3d(0,1.9)
#            plt.show()
            
#            u.append(thrust)
            ''' !!! treba da se nacrta i pozicijata i da se sporedi so i bez consensus !!! '''
            xl.append(pos[0])
            xf2.append(pos2[0])
            xf0.append(pos0[0])
            yl.append(pos[1])
            yf2.append(pos2[1])
            yf0.append(pos0[1])
            zl.append(pos[2])
            zf2.append(pos2[2])
            zf0.append(pos0[2])
#            vxl.append(vx1[2])
#            vxf.append(vx2[2])
#            vyl.append(vy1[2])
#            vyf.append(vy2[2])
#            euler_l.append(euler[2])
#            euler_f.append(euler2[2])
#            v1.append(particlesTargetVelocities[0])
#            v2.append(particlesTargetVelocities[1])
#            v3.append(particlesTargetVelocities[2])
#            v4.append(particlesTargetVelocities[3])
#            v12.append(particlesTargetVelocities2[0])
#            v22.append(particlesTargetVelocities2[1])
#            v32.append(particlesTargetVelocities2[2])
#            v42.append(particlesTargetVelocities2[3])
            plt.plot(zl,color='blue')
            plt.hold(True)
            plt.plot(zf2,color='red')
            plt.hold(True)
            plt.plot(zf0,color='pink')
            plt.axis([0,449,0,2.1])
            plt.show()
            plt.plot(xl,color='blue')
            plt.hold(True)
            plt.plot(xf2,color='red')
            plt.hold(True)
            plt.plot(xf0,color='pink')
            plt.axis([0,449,-3,3])
            plt.show()
            plt.plot(yl,color='blue')
            plt.hold(True)
            plt.plot(yf2,color='red')
            plt.hold(True)
            plt.plot(yf0,color='pink')
            plt.axis([0,449,-3,3])
            plt.show()
#            print " Zaxis error between drones: ", pos[2]-pos2[2]
            ze2.append(pos[2]-pos2[2])
            ze0.append(pos[2]-pos0[2])
            xe2.append(pos[0]-pos2[0])
            xe0.append(pos[0]-pos0[0])
            ye2.append(pos[1]-pos2[1])
            ye0.append(pos[1]-pos0[1])
#            plt.plot(ze2,color='green')
#            plt.hold(True)
#            plt.plot(ze0,color='black')
#            plt.axis([0,200,-3,3])
#            plt.show()
            """ =========================================================== """
            """ ========== Gradient descent: ========== """
#            sum_h_alpha.append(cumulAlpha)
#            sum_h_beta.append(cumulBeta)
#            sum_h_pos1.append(cumulAlphaPos)
#            sum_h_pos0.append(cumulBetaPos)
            
#            J_h_alpha.append((1/(tf - t0))*(5*cumulAlpha**2 + delta_alpha_angle[gd]**2)*dt)
#            J_h_beta.append((1/(tf - t0))*(5*cumulBeta**2 + delta_beta_angle[gd]**2)*dt)
#            J_h_pos1.append((1/(tf - t0))*(5*cumulAlphaPos**2 + delta_alpha_pos[gd]**2)*dt)
#            J_h_pos0.append((1/(tf - t0))*(5*cumulBetaPos**2 + delta_beta_pos[gd]**2)*dt)
            
#            paramX.append(targetPos[0])
#            paramY.append(targetPos[1])
            
#            print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
#            print "theta_h_angles : ", theta_h_angles
#            print "theta_h_pos : ", theta_h_pos
#            
#            print "sum_h_alpha : ", sum_h_alpha[gd]
#            print "sum_h_beta : ", sum_h_beta[gd]
#            print "sum_h_pos1 : ", sum_h_pos1[gd]
#            print "sum_h_pos0 : ", sum_h_pos0[gd]
            
#            print "J_h_alpha : ", J_h_alpha[gd]
#            print "J_h_beta : ", J_h_beta[gd]
#            print "J_h_pos1 : ", J_h_pos1[gd]
#            print "J_h_pos0 : ", J_h_pos0[gd]
            
#            print "paramX : ", paramX[gd]
#            print "paramY : ", paramY[gd]
#            print "cumulPOS : ", cumulPOS2
#            print "cumulVY : ", cumulVY2
#            print "cumulVX : ", cumulVX2
#            print "cumulSP0 : ", cumulSP02
#            print "cumulSP1 : ", cumulSP12
#            print "cumulEULER : ", cumulEULER2
#            print "cumul : ", cumul
            print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
            gd = gd + 1
            print "gd (iterations) : ", gd
            print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
            """ =========================================================== """
            if gd == 450:
#                with open('GD_43.csv','wb') as f1:
#                    writer=csv.writer(f1)
#                    for i in range(0, gd):
#                        writer.writerow([paramX[i],paramY[i],theta_h_angles[0],theta_h_angles[1],theta_h_angles[2],theta_h_pos[0],theta_h_pos[1],theta_h_pos[2],J_h_alpha[i],J_h_beta[i],J_h_pos1[i],J_h_pos0[i]])
                break
            
#            """ =========================================================== """
            """ ========== WRITE TO TEXT: ========== """
            ## WRITE TO TEXT:
#            log_data(C_parameters_vert, C_parameters_horr, C_parameters_rot, vert_comp, horr_comp, rot_comp, particlesTargetVelocities, i)
#            i +=1
            """ =========================================================== """
           # send propeller velocities to output:
            [res,retInts,retFloats,retStrings,retBuffer]=vrep.simxCallScriptFunction(clientID,Quadricopter,vrep.sim_scripttype_childscript,'qc_propeller_v',[],particlesTargetVelocities_s,[],emptyBuff,vrep.simx_opmode_blocking)   
#            [res,retInts,retFloats,retStrings,retBuffer]=vrep.simxCallScriptFunction(clientID,Quadricopter2,vrep.sim_scripttype_childscript,'qc_propeller_v',[],particlesTargetVelocities2,[],emptyBuff,vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            """ =========================================================== """
#        print " Zaxis error between drones: ", ze
        print "Z Error between leader-2", ze2
        print "Z Error between leader-0", ze0
        print "X Error between leader-2", xe2
        print "X Error between leader-0", xe0
        print "Y Error between leader-2", ye2
        print "Y Error between leader-0", ye0
        # stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
        
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
            
    else:
        print ('Failed connecting to remote API server')
        