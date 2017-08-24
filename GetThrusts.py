def GetVelocities(thrust, alphaCorr, betaCorr, rotCorr):
    propeller1_PTV = thrust*(1 - alphaCorr + betaCorr + rotCorr)
    propeller2_PTV = thrust*(1 - alphaCorr - betaCorr - rotCorr)
    propeller3_PTV = thrust*(1 + alphaCorr - betaCorr + rotCorr)
    propeller4_PTV = thrust*(1 + alphaCorr + betaCorr - rotCorr)
    particlesTargetVelocities = [propeller1_PTV, propeller2_PTV, propeller3_PTV, propeller4_PTV]
    
    return particlesTargetVelocities