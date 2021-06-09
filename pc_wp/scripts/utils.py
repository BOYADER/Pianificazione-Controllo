#!/usr/bin/env python
import numpy as np
import math
import rospy
from termcolor import colored
from classes.Waypoint import Waypoint

def wrap2pi(angle):
		return np.arctan2(np.sin(angle), np.cos(angle))					# return angle between [-pi, pi]

def isNone(vars_array):										# check if there's one var = None in the array
	ans = False
	for var in vars_array:
		if var is None:
			ans = True
	return ans

def clear(vars_array):										# set all vars to None in the array
	None_array = []
	for var in vars_array:
		None_array.append(None)
	return None_array

def get_waypoint(index):									# return waypoint
	if index < len(rospy.get_param('waypoint_list')):					# from waypoint list in mission.yaml file
		lld_ned = rospy.get_param('ned_frame_origin')
		string_param = 'waypoint_list/wp' + str(index + 1)			
		latitude = rospy.get_param(string_param)['latitude']
		longitude = rospy.get_param(string_param)['longitude']
		depth = rospy.get_param(string_param)['depth']
		waypoint = Waypoint(latitude, longitude, depth, lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
		return waypoint
	elif index == len(rospy.get_param('waypoint_list')):					# all of the waypoint in the list are approached, return the ned origin
		lld_ned = rospy.get_param('ned_frame_origin')
		waypoint = Waypoint(lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'], lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
		return waypoint
	else:
		return None

def ned2body(pos_ned, eta_2):									# ned to body transformation
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
	R = np.dot(np.dot(R_x, R_y), R_z)
	pos_body = np.dot(R, pos_ned)
	return [pos_body[0], pos_body[1], pos_body[2]]

def projection(u, v):										# projection of u over v
	proj = (np.dot(u, v)/np.linalg.norm(v)**2)*v
	return proj

def isOverSetPoint(set_point, actual_value, sign):						# check if actual_value is over set_point
	if (	(set_point >= 0 and actual_value >= 0 and sign == 1 and actual_value >= set_point) or 
		(set_point >= 0 and actual_value >= 0 and sign == -1 and actual_value <= set_point) or 
		(set_point <= 0 and actual_value <= 0 and sign == 1 and actual_value >= set_point) or 
		(set_point <= 0 and actual_value <= 0 and sign == -1 and actual_value <= set_point)	):   
		return True
	else:
		return False

def print_info(references, state, eta_1, eta_2, ni_1, control):					# print references, eta_1, eta_2, ni_1, forces, torques on screen
	if state.wp_index == len(rospy.get_param('waypoint_list')):
		string_adv = "Back to NED origin"			
	else:
		string_adv = "Waypoint n. " + str(state.wp_index + 1)
	print("#############################################################################################################\n")
	print("                                                %s                                                           \n" %  string_adv)	
	print("#############################################################################################################")
	print("-------------------------------------------------------------------------------------------------------------")	
	string_param = 'task_seq_list/ts' + str(state.strategy)
	task_seq = rospy.get_param(string_param)
	string_print = "                                    "
	for x in range(0, len(task_seq)):
		if task_seq[x] == state.task:
			task_colored = colored(state.task, 'green', attrs=['bold'])		# set green and bold the current task
			string_print = string_print + task_colored
		else:
			string_print = string_print + task_seq[x]
		if x < (len(task_seq) - 1):
				string_print = string_print + ' - '
	print(string_print)									# print the task sequence on screen
	print("-------------------------------------------------------------------------------------------------------------")
	print("                            actual values                    references                      measure unit    ")
	print("-------------------------------------------------------------------------------------------------------------")
	print("  eta_1.x			%s				%s				[m]         " % (round(eta_1[0], 2), round(references.pos.x, 2)))
	print("  eta_1.y			%s				%s				[m]         " % (round(eta_1[1], 2), round(references.pos.y, 2)))
	print("  eta_1.z			%s				%s  				[m]         " % (round(eta_1[2], 2), round(references.pos.z, 2)))
	print("-------------------------------------------------------------------------------------------------------------")
	print("  eta_2.x			%s								[deg]       " % (round(math.degrees(eta_2[0]), 1)))
	print("  eta_2.y			%s				%s				[deg]       " % (round(math.degrees(eta_2[1]), 1), round(math.degrees(references.rpy.y), 1)))
	print("  eta_2.z			%s				%s				[deg]       " % (round(math.degrees(eta_2[2]), 1), round(math.degrees(references.rpy.z), 1)))
	print("-------------------------------------------------------------------------------------------------------------")
	if state.task != 'SURGE':
		print("  ni_1.x 			%s								[m/s]       " % (round(ni_1[0], 2)))
	else:
		print("  ni_1.x 			%s				%s				[m/s]       " % (round(ni_1[0], 2), round(references.lin_vel.x, 2)))
	print("  ni_1.y 			%s								[m/s]       " % (round(ni_1[1], 2)))
	print("  ni_1.z 			%s								[m/s]       " % (round(ni_1[2], 2)))
	print("-------------------------------------------------------------------------------------------------------------")
	print("  force.x			%s								[N]	    " % round(control.tau.force.x, 2))
	print("  force.y			%s								[N]	    " % round(control.tau.force.y, 2))
	print("  force.z			%s								[N]	    " % round(control.tau.force.z, 2))
	print("-------------------------------------------------------------------------------------------------------------")
	print("  torque.x			%s								[Nm]	    " % round(control.tau.torque.x, 2))
	print("  torque.y			%s								[Nm]	    " % round(control.tau.torque.y, 2))
	print("  torque.z			%s								[Nm]	    " % round(control.tau.torque.z, 2))
	print("-------------------------------------------------------------------------------------------------------------")

def print_end_mission():									# print mission completed at the end of the mission on screen
	print("\n")
	print("        __  __ ___ ____ ____ ___ ___  _   _     ____ ___  __  __ ____  _     _____ _____ _____ ____  ")
	print("       |  \/  |_ _/ ___/ ___|_ _/ _ \| \ | |   / ___/ _ \|  \/  |  _ \| |   | ____|_   _| ____|  _ \ ")
	print("       | |\/| || |\___ \___ \| | | | |  \| |  | |  | | | | |\/| | |_) | |   |  _|   | | |  _| | | | |")
	print("       | |  | || | ___) |__) | | |_| | |\  |  | |__| |_| | |  | |  __/| |___| |___  | | | |___| |_| |")
	print("       |_|  |_|___|____/____/___\___/|_| \_|   \____\___/|_|  |_|_|   |_____|_____| |_| |_____|____/ ")
	print("\n")

	
	
