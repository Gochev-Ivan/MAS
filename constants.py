""" CONSTANTS AND PARAMETERS FOR QC SCRIPTS: """
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