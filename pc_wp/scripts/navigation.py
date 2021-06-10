#!/usr/bin/env python
import rospy
import math
from pc_wp.msg import Odom
from modellazione.msg import state_real
import pymap3d as pm

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def navigation():
	rospy.init_node('navigation')
	pub = rospy.Publisher('odom', Odom, queue_size = QUEUE_SIZE)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		data = rospy.wait_for_message('state_real', state_real, timeout=None)
		odom_msg = Odom()
		odom_msg.rpy.x = data.eta_2.x
		odom_msg.rpy.y = data.eta_2.y
		odom_msg.rpy.z = data.eta_2.z
		odom_msg.lin_vel.x = data.ni_1.x
		odom_msg.lin_vel.y = data.ni_1.y
		odom_msg.lin_vel.z = data.ni_1.z
		lld_ned = rospy.get_param('initial_pose')['position']
		odom_msg.lld.x = pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0]
		odom_msg.lld.y = pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1]
		odom_msg.lld.z = -pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]
		pub.publish(odom_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
