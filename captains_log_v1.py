import time
import csv
import os

def log_data(C_parameters_vert, C_parameters_horr, C_parameters_rot, vert_comp, horr_comp, rot_comp, particlesTargetVelocities, i):
    Kpv = C_parameters_vert[0]
    Kiv = C_parameters_vert[1]
    Kdv = C_parameters_vert[2]
    Kph = C_parameters_horr[0]
    Kih = C_parameters_horr[1]
    Kdh = C_parameters_horr[2]
    Kph_pos0 = C_parameters_horr[3]
    Kdh_pos0 = C_parameters_horr[4]
    Kph_pos1 = C_parameters_horr[5]
    Kdh_pos1 = C_parameters_horr[6]
    Kpr = C_parameters_rot[0]
    Kir = C_parameters_rot[1]
    Kdr = C_parameters_rot[2]
    targetPos = vert_comp[0]
    pos = vert_comp[1]
    e = vert_comp[2]
    thrust = vert_comp[3]
    alphaE = horr_comp[0]
    betaE = horr_comp[1]
    cumulAlpha = horr_comp[2]
    cumulBeta = horr_comp[3]
    sp = horr_comp[4]
    cumulAlphaPos = horr_comp[5]
    cumulBetaPos = horr_comp[6]
    alphaCorr = horr_comp[7]
    betaCorr = horr_comp[8]
    vx = horr_comp[9]
    vy = horr_comp[10]
    euler = rot_comp[0]
    rotCorr = rot_comp[1]
    
#    print '////////////'
#    print '============'
#    print '-Controller parameters-'
#    print 'Vertical control parameters=',[Kpv,Kiv,Kdv]
#    print 'Horizontal control parameters=',[Kph,Kih,Kdh,Kph_pos0,Kdh_pos0,Kph_pos1,Kdh_pos1]
#    print 'Rotational control parameters=',[Kpr,Kir,Kdr]
#    print '============'
#    print '-Vertical component-'
#    print 'targetPos=',targetPos
#    print 'pos=',pos        
#    print 'e=',e
#    print 'thrust=',thrust
#    print '============'
#    print '-Horizontal component-'
#    print 'vx=',vx
#    print 'vy=',vy
#    print 'alphaE=',alphaE
#    print 'betaE=',betaE
#    print 'cumulAlpha=',cumulAlpha
#    print 'cumulBeta=',cumulBeta
#    print 'sp_X=',sp[0]
#    print 'sp_Y=',sp[1]
#    print 'cumulAlphaPos=',cumulAlphaPos
#    print 'cumulBetaPos=',cumulBetaPos
#    print 'alphaCorr=',alphaCorr
#    print 'betaCorr=',betaCorr
#    print '============'
#    print '-Rotational component-'
#    print 'gamma=',euler[2]
#    print 'rotCorr=',rotCorr
#    print '============'
#    print 'Velocity_propeller_1=',particlesTargetVelocities[0]
#    print 'Velocity_propeller_2=',particlesTargetVelocities[1]
#    print 'Velocity_propeller_3=',particlesTargetVelocities[2]
#    print 'Velocity_propeller_4=',particlesTargetVelocities[3]
#    print '============'
#    print '////////////'
    
    ## WRITE TO TXT:
    filepath = os.path.join('C:\Users\Ivan\Dropbox\Fax\NCS_prodolzenie\python_simulaciona_okolina_v2\python_and_vrep_v2\captains_log\sim5', "test"+str(i)+".txt")
    text_file=open(filepath,'w')
    
    text_file.write('Vertical control parameters='+str([Kpv,Kiv,Kdv]))
    text_file.write("\n")
    text_file.write('Horizontal control parameters='+str([Kph,Kih,Kdh,Kph_pos0,Kdh_pos0,Kph_pos1,Kdh_pos1]))
    text_file.write("\n")
    text_file.write('Rotational control parameters='+str([Kpr,Kir,Kdr]))    
    text_file.write("\n")
    text_file.write('targetPos='+str(targetPos))
    text_file.write("\n")
    text_file.write('pos='+str(pos))
    text_file.write("\n")
    text_file.write('e='+str(e))
    text_file.write("\n")
    text_file.write('thrust='+str(thrust))
    text_file.write("\n")
    
    text_file.write('vx='+str(vx))
    text_file.write("\n")
    text_file.write('vy='+str(vy))
    text_file.write("\n")
    text_file.write('alphaE='+str(alphaE))
    text_file.write("\n")
    text_file.write('betaE='+str(betaE))
    text_file.write("\n")
    text_file.write('cumulAlpha='+str(cumulAlpha))
    text_file.write("\n")
    text_file.write('cumulBeta='+str(cumulBeta))
    text_file.write("\n")
    text_file.write('sp_X='+str(sp[0]))
    text_file.write("\n")
    text_file.write('sp_Y='+str(sp[1]))
    text_file.write("\n")
    text_file.write('cumulAlphaPos='+str(cumulAlphaPos))
    text_file.write("\n")
    text_file.write('cumulBetaPos='+str(cumulBetaPos))
    text_file.write("\n")
    text_file.write('alphaCorr='+str(alphaCorr))
    text_file.write("\n")
    text_file.write('betaCorr='+str(betaCorr))
    text_file.write("\n")
    
    text_file.write('gamma='+str(euler[2]))
    text_file.write("\n")
    text_file.write('rotCorr='+str(rotCorr))
    text_file.write("\n")
    
    text_file.write('Velocity_propeller_1='+str(particlesTargetVelocities[0]))
    text_file.write("\n")
    text_file.write('Velocity_propeller_2='+str(particlesTargetVelocities[1]))
    text_file.write("\n")
    text_file.write('Velocity_propeller_3='+str(particlesTargetVelocities[2]))
    text_file.write("\n")
    text_file.write('Velocity_propeller_4='+str(particlesTargetVelocities[3]))
    text_file.write("\n")
    
#    text_file.write('Simulation_time='+str(t_end-time.time()))
#    text_file.write("\n")
    text_file.write("////////////")
    text_file.write("\n")
    
    ## WRITE TO CSV
#    with open('test_list_Astar_14.csv','wb') as f1:
#        writer=csv.writer(f1)
#        for i in range(0, len()):
#            writer.writerow([])
    return
    
    
def log_data_for_GD():
    return