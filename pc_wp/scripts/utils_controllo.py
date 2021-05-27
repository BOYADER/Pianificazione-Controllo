#!/usr/bin/env python
import numpy as np
import math
import rospy
from classes.Waypoint import Waypoint


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


