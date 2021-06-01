#!/usr/bin/env python
import rospy
import math
from pc_wp.msg import Odom
from modellazione.msg import state_real
import pymap3d as pm

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def state_callback(data, pub):
	odom_msg = Odom()
	odom_msg.rpy.x = round(data.eta_2.x,5)
	odom_msg.rpy.y = round(data.eta_2.y,5)
	odom_msg.rpy.z = round(data.eta_2.z,5)
	odom_msg.lin_vel.x = round(data.ni_1.x, 5)
	odom_msg.lin_vel.y = round(data.ni_1.y, 5)
	odom_msg.lin_vel.z = round(data.ni_1.z, 5)
	lld_ned = rospy.get_param('initial_pose')['position']
	odom_msg.lld.x = round(pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0],5)
	odom_msg.lld.y = round(pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1], 5)
	odom_msg.lld.z = round(-pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2],5)
	pub.publish(odom_msg)

def navigation():
	rospy.init_node('navigation')
	pub = rospy.Publisher('odom', Odom, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state_real', state_real, state_callback, pub, queue_size = 1)
	rospy.spin()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
