""" MAIN SCRIPT FOR CALLING FUNCTIONS AND EXECUTING """

import QC_sync_funcion_v0
import Quadricopter_main_script_backup
import main_qc_algorithms_testing_script
import main_qc_algorithms_testing_script_backup
import main_qc_algorithms_testing_script_3Drones
import main_qc_algorithms_testing_script_4Drones

if __name__ == "__main__":
    drone_targets = ['Quadricopter_target','Quadricopter_target2','Quadricopter_target0','Quadricopter_target1']
    bases = ['Quadricopter_base','Quadricopter_base2','Quadricopter_base0','Quadricopter_base1']
    fleet = ['Quadricopter','Quadricopter2','Quadricopter0','Quadricopter1']
    
    """ QC_controller(Quadricopter_target, Quadricopter_base, Quadricopter) """
##    while True:
    
#    QC_sync_funcion_v0.QC_controller(drone_targets[0], bases[0], fleet[0])
#    print drone_targets[1], bases[1], fleet[1]
    
#    QC_sync_funcion_v0.QC_controller(drone_targets[1], bases[1], fleet[1])
#    QC_sync_funcion_v0.QC_controller(drone_targets[2], bases[2], fleet[2])
#    QC_sync_funcion_v0.QC_controller(drone_targets[3], bases[3], fleet[3])
    
    
    print drone_targets[1], bases[1], fleet[1]
#    main_qc_algorithms_testing_script.QC_controller(drone_targets[0], bases[0], fleet[0], drone_targets[1], bases[1], fleet[1])
#    main_qc_algorithms_testing_script_3Drones.QC_controller(drone_targets[0], bases[0], fleet[0], drone_targets[1], bases[1], fleet[1], drone_targets[2], bases[2], fleet[2])
    main_qc_algorithms_testing_script_4Drones.QC_controller(drone_targets[0], bases[0], fleet[0], drone_targets[1], bases[1], fleet[1], drone_targets[2], bases[2], fleet[2],drone_targets[3], bases[3], fleet[3])
#    main_qc_algorithms_testing_script_backup.QC_controller(drone_targets[0], bases[0], fleet[0])
    
    
    
    ''' =================================================================== '''
    # test za shtimanje na PID preku Quadricopter_main_script
#    while True:
#    Quadricopter_main_script_backup.QC_controller(drone_targets[0], bases[0], fleet[0])