
def PID(targetPos, pos, cumul, last_e, Kpv, Kiv, Kdv, vParam, l, vy, m, cumulAlpha, pAlphaE, Kph, Kih, Kdh, vx, cumulBeta, pBetaE, 
        cumulAlphaPos, cumulBetaPos, sp, Kph_pos1, Kih_pos1, Kdh_pos1, Kph_pos0, Kih_pos0, Kdh_pos0, psp2, psp1, Kpr, Kdr, euler, 
        prevEuler, cumulPOS, cumulVY, cumulVX, cumulSP0, cumulSP1, cumulEULER, vx2, vy2, pos2, sp2, euler2):  
    
    if (pos[2] - pos2[2]) == 0:
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
    else:
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
    
    """ ========== vertical control: ========== """
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
    alphaE=vy[2]-m#[11]
    cumulAlpha = cumulAlpha + alphaE
    diff_alphaE=alphaE-pAlphaE
    alphaCorr=Kph*alphaE + Kih*cumulAlpha + Kdh*diff_alphaE #alpha correction
    
    betaE=vx[2]-m#[11]
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
    
    """ =========================================================== """            
    """ ========== rotational control: ========== """    
    Prot=Kpr*euler[2]
    Drot=Kdr*(euler[2]-prevEuler)
    rotCorr=Prot+Drot
    prevEuler=euler[2]
    
    """ =========================================================== """
    """ ========== CONSENSUS: ========== """
    # pos2[2] ; vx2[2](za beta) ; vy2[2](za alpha) ; sp2[1]->y ; sp2[0]->x ; euler2[2] -> i
    # pos[2] ; vx[2](za beta) ; vy[2](za alpha) ; sp[1]->y ; sp[0]->x ; euler[2] -> j/l(leader)
    
    cumulPOS = cumulPOS + 0.0001*(-pos[2] + pos2[2]) # TEZHINSKIOT KOEFICIENT ZA CONSENSUSOT TREBA DA SE NASHTIMA
    cumulVY = cumulVY + 0.0001*(-vy[2] + vy2[2])
    cumulVX = cumulVX + 0.0001*(-vx[2] + vx2[2])
#    cumulSP0 = cumulSP0 + 0.001*(-sp[0] + sp2[0])
#    cumulSP1 = cumulSP1 + 0.001*(-sp[1] + sp2[1])
    cumulSP0 = cumulSP0 + 0.0001*(-pos[0] + pos2[0])
    cumulSP1 = cumulSP1 + 0.0001*(-pos[1] + pos2[1])
#    cumulEULER = cumulEULER + 0.001*(-euler[2] + euler2[2]) # samo PD upravuvach
    
#    cumulPOS = 0.1*(-pos[2] + pos2[2]) # TEZHINSKIOT KOEFICIENT ZA CONSENSUSOT TREBA DA SE NASHTIMA
#    cumulVY = 0.1*(-vy[2] + vy2[2])
#    cumulVX = 0.1*(-vx[2] + vx2[2])
#    cumulSP0 = 0.1*(-sp[0] + sp2[0])
#    cumulSP1 = 0.1*(-sp[1] + sp2[1])
#    cumulEULER = 0.1*(-euler[2] + euler2[2])
    #    if (pos[2] - pos2[2]) <= (pos[2] - pos2[2])/1000:
#        cumulPOS = 0
#    if (-vy[2] + vy2[2]) <= (-vy[2] + vy2[2])/100:
#        cumulVY = 0
#    if (-vx[2] + vx2[2]) <= (-vx[2] + vx2[2])/100:
#        cumulVX = 0
#    if (-sp[0] + sp2[0]) <= (-sp[0] + sp2[0])/100:
#        cumulSP0 = 0
#    if (-sp[1] + sp2[1]) <= (-sp[1] + sp2[1])/100:
#        cumulSP1 = 0
#    if (-euler[2] + euler2[2]) <= (-euler[2] + euler2[2])/100:
#        cumulEULER = 0
    thrust = thrust + cumulPOS
    alphaCorr = alphaCorr + cumulVY
    betaCorr = betaCorr + cumulVX
    betaPos = betaPos + cumulSP0
    alphaPos = alphaPos + cumulSP1
    rotCorr = rotCorr + cumulEULER
    
#    print " position error: ", pos[2] - pos2[2]
#    print " vy error: ", -vy[2] + vy2[2]
#    print " vx error: ", -vx[2] + vx2[2]
#    print " sp0 error: ", -sp[0] + sp2[0]
#    print " sp1 error: ", -sp[1] + sp2[1]
#    print " euler error: ", -euler[2] + euler2[2]
#    print "cumulPOS : ", cumulPOS
#    print "cumulVY : ", cumulVY
#    print "cumulVX : ", cumulVX
#    print "cumulSP0 : ", cumulSP0
#    print "cumulSP1 : ", cumulSP1
#    print "cumulEULER : ", cumulEULER
#    print "=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*="
    """ =========================================================== """
    """ ========== set propeller velocities: ========== """
    propeller1_PTV = thrust*(1-alphaCorr+betaCorr-rotCorr)
    propeller2_PTV = thrust*(1-alphaCorr-betaCorr+rotCorr)
    propeller3_PTV = thrust*(1+alphaCorr-betaCorr-rotCorr)
    propeller4_PTV = thrust*(1+alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities=[propeller1_PTV, propeller2_PTV, propeller3_PTV, propeller4_PTV]
    
    return particlesTargetVelocities, cumul, last_e, cumulAlpha, pAlphaE, cumulBeta, pBetaE, cumulAlphaPos, cumulBetaPos, psp2, psp1, prevEuler, cumulPOS, cumulVY, cumulVX, cumulSP0, cumulSP1, cumulEULER
    """ =========================================================== """
    
    
    
    
    
    
    
    
    