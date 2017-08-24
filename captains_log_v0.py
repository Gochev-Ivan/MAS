from pythonvrep_test_remoteapi_v3 import qc_script
import time

#t_end = time.time() + 5
#
#
#speed weight:
vParam=-2
#parameters for vertical control
Kpv=2
Kiv=0
Kdv=1.1
#parameters for horizontal control
Kph=1.5#0.9
Kih=0
Kdh=2.125#1.1
Kph_pos1=1#0.1
Kdh_pos1=1.2#0.05
Kph_pos0=1#0.1
Kdh_pos0=1.2#0.05
#parameters for rotational control:
Kpr=0.1
Kir=0
Kdr=1.1

logIterator=0

#
#
#while time.time() < t_end:
qc_script(Kpv,Kiv,Kdv,Kph,Kih,Kdh,Kph_pos0,Kdh_pos0,Kph_pos1,Kdh_pos1,Kpr,Kir,Kdr,vParam,logIterator)
    
#a=[1,2,3]    
#for i in range(55,56):
#   text_file=open("test"+str(i)+".txt",'w')
#   text_file.write('a='+str(a))
#   text_file.write("\n")
#   text_file.write('a='+str(a))
#   text_file.close() 

