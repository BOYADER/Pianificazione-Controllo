#!/usr/bin/env python
import numpy as np
import math
import rospy
from classes.Waypoint import Waypoint


#funzione che legge il task corrente e calcola il vettore colonna error in ned da mandare alla funzione error_ned2body  (la funzione set_mini_ref e' sotto per chi se lo stesse chiedendo)
def set_error(current_task, ref,eta_1, eta_2): #passati o messi global?
    if current_task == 'YAW':
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = ref.pos.z - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(set_mini_ref(ref.rpy.z) - eta_2[2])
    if current_task == 'PITCH':
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = ref.pos.z - eta_1[2]
        error_pitch = wrap2pi(set_mini_ref(ref.rpy.y) - eta_2[1])
        error_yaw = wrap2pi(ref.rpy.z - eta_2[2])

    if current_task == 'SURGE': #TODO
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = ref.pos.z - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(set_yaw_ref(ref.rpy.z) - eta_2[2])
        
    if current_task == 'HEAVE':
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = set_mini_ref(ref.pos.z) - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(ref.rpy.z - eta_2[2])
    if current_task == 'APPROACH': #TODO
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = set_mini_ref(ref.pos.z) - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(ref.rpy.z - eta_2[2])
        
    #alla fine metto tutto in un vettore colonna
    error = np.array([[error_x, error_y, error_z, error_yaw, error_pitch, error_roll]]).T
    return error

#funzione che legge lo stato e il riferimento finale e sputa fuori il mini riferimento
def set_mini_ref(final_value, state):
    global eta_2, time_start, task_changed, current_task
    yaw_angular_velocity = rospy.get_param('/yaw_angular_velocity')
    pitch_angular_velocity = rospy.get_param('/pitch_angular_velocity')
    x_linear_velocity = rospy.get_param('/x_linear_velocity')
    z_linear_velocity = rospy.get_param('/z_linear_velocity')
    if state.task == 'YAW':
        if not time_start or task_changed:
            time_start = time.time()  # init time_start
            task_changed = False
        dt = time.time() - time_start
        ref = eta_2[2] + np.sign(final_value - eta_2[2]) * yaw_angular_velocity * dt
    if state.task == 'PITCH':
        if not time_start or task_changed:
            time_start = time.time()  # init time_start
            task_changed = False
        dt = time.time() - time_start
        ref = eta_2[1] + np.sign(final_value - eta_2[1]) * pitch_angular_velocity  * dt
    if state.task == 'HEAVE':
        if not time_start or task_changed:
            time_start = time.time()  # init time_start
            task_changed = False
        dt = time.time() - time_start
        ref = eta_1[2] + np.sign(final_value - eta_1[2]) * z_linear_velocity  * dt
    if state.task == 'SURGE': #TODO
        if not time_start or task_changed:
            time_start = time.time()  # init time_start
            task_changed = False
        dt = time.time() - time_start
        ref = eta_1[2] + np.sign(final_value - eta_1[2]) * z_linear_velocity  * dt
    if state.task == 'APPROACH': #TODO
        if not time_start or task_changed:
            time_start = time.time()  # init time_start
            task_changed = False
        dt = time.time() - time_start
        ref = eta_1[2] + np.sign(final_value - eta_1[2]) * z_linear_velocity  * dt
       
    #infine ritorno il valore del miniriferimento
    if abs(final_value) < abs(ref):
        return final_value
    else:
        return ref


#funzione che prende error in ned e restituisce l'errore in body
def error_ned2body(error, eta_2):  #error ed eta_2 sono due vettori colonna error = np.array([[error_x, error_y, error_z, error_roll, error_pitch, error_yaw]]).T
    # definisco J1(eta2)
    roll = eta_2[0]
    pitch = eta_2[1]
    yaw = eta_2[2]
    R_x_roll = np.array([[1, 0, 0],
                        [0, math.cos(roll), math.sin(roll)],
                        [0, -math.sin(roll), math.cos(roll)]])
    R_y_pitch = np.array([[math.cos(pitch), 0, -math.sin(pitch)],
                         [0, 1, 0],
                         [math.sin(pitch), 0, math.cos(pitch)]])
    R_z_yaw = np.array([[math.cos(yaw), math.sin(yaw), 0],
                        [-math.sin(yaw), math.cos(yaw), 0],
                        [0, 0, 1]])
    R_z_t = np.transpose(R_z_yaw)
    R_y_t = np.transpose(R_y_pitch)
    R_x_t = np.transpose(R_x_roll)
    J1 = np.dot(np.dot(R_z_t, R_y_t), R_x_t)

    error_xyz = error[0:3] #spacchetto errore nelle prime 3 e ultime 3 componenti
    error_rpy = error[3:7]
    error_xyz_body = np.dot(np.transpose(J1), error_xyz)     #trasformo le prime 3 in body
    error_body = np.array([error_xyz_body, error_rpy]) #rimetto tutto insieme
    return error_body
  
#funzione che prende in ingresso l'errore in body e sputa fuori le tau da pubblicare sul topic wrench
def controllo(error_body):                             #tutte cose da definire come parametri in un file yaml
    global int_error, dt, UP_SAT, DOWN_SAT, K_P, K_I  #integrale dell'errore, intervallo di tempo, saturazione superiore, saturazione inferiore, matrice dei guadagni P, matrice dei guadagni I 
    int_error = int_error + (error_body * dt)     #calcolo integrale dell'errore
    u = np.dot(K_P, error_body) + np.dot(K_I, int_error) #controllore
   # controllo che nessuna tau superi la saturazione superiore o inferiore
    for i in range(0, 6):
        if u[i] > UP_SAT:
            u[i] = UP_SAT
            int_error = int_error - (error_body * dt) #anti reset windup
        elif u[i] < DOWN_SAT:
            u[i] = DOWN_SAT
            int_error = int_error - (error_body * dt)
    return u


