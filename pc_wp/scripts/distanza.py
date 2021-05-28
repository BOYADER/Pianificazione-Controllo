#!/usr/bin/env python
import math
import numpy as np

def ned2body(vector_ned, eta_2):
	roll = eta_2[0]
	pitch = eta_2[1]
	yaw = eta_2[2]
	R_x = np.array([	[1, 0, 0],
		     		[0, math.cos(roll), math.sin(roll)],
	                 	[0, -math.sin(roll), math.cos(roll)]])
	R_y = np.array([	[math.cos(pitch), 0, -math.sin(pitch)],
				[0, 1, 0],
				[math.sin(pitch), 0, math.cos(pitch)]])
	R_z = np.array([	[math.cos(yaw), math.sin(yaw), 0],
                        	[-math.sin(yaw), math.cos(yaw), 0],
                        	[0, 0, 1]])
	R_z_t = np.transpose(R_z)
	R_y_t = np.transpose(R_y)
	R_x_t = np.transpose(R_x)
	J1 = np.dot(np.dot(R_z_t, R_y_t), R_x_t)
   	vector_pos_ned = [	vector_ned[0],
				vector_ned[1],
				vector_ned[2]]
	vector_pos_body = np.dot(np.transpose(J1), vector_pos_ned)
	return np.array([vector_pos_body])


wp1 = np.array([6,5, 5]) 
wp2 = np.array([21, 11, 10]) 

auv = np.array([9,10,7])

lui1 = auv - wp1
lui2 = wp2 - wp1

print("lui1: %s, lui2: %s" % (lui1, lui2))

lui2_norm = np.sqrt(sum(lui2**2))    
  
lui1_on_lui2 = (np.dot(lui1, lui2)/lui2_norm**2)*lui2
print("lui1_on_lui2: %s" % lui1_on_lui2)

lui4 = lui1_on_lui2-lui1
print("lui4: %s" % lui4)
lui4_norm = np.sqrt(sum(lui4**2)) 
print("lui4_norm: %s" % lui4_norm)

print("QUESTO: %s" % (wp1+lui4+lui1))

print("errore su pianoxy: %s" % np.sqrt(lui4[0]**2+lui4[1]**2))
print("errore su pianoyz: %s" % np.sqrt(lui4[1]**2+lui4[2]**2))
print("errore su pianoxz: %s" % np.sqrt(lui4[0]**2+lui4[2]**2))

#print(ned2body(lui1-lui1_on_lui2, lui1))
