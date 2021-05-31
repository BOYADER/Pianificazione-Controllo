#!/usr/bin/env python
import rospy
import math
from pc_wp.msg import Odom
from modellazione.msg import state_real
import pymap3d as pm

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def state_callback(data, pub):
	odom_msg = Odom()
	odom_msg.rpy = data.eta_2
	odom_msg.lin_vel = data.ni_1
	lld_ned = rospy.get_param('initial_pose')['position']
	odom_msg.lld.x = pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0]
	odom_msg.lld.y = pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1]
	odom_msg.lld.z = pm.ned2geodetic(data.eta_1.x, data.eta_1.y, data.eta_1.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]
	
	pub.publish(odom_msg)
	#print(odom_msg)

def navigation():
	rospy.init_node('navigation')
	pub = rospy.Publisher('odom', Odom, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state_real', state_real, state_callback, pub)

	#pub = rospy.Publisher('odom', Odom, queue_size = QUEUE_SIZE)
	rospy.loginfo("navigation node started")
	"""rate = rospy.Rate(1) # 1hz
	odom_msg = Odom()
	odom_msg.lld.x = 45.110735 
	odom_msg.lld.y = 7.640827 
	odom_msg.lld.z = 0
	odom_msg.rpy.x = math.radians(0)
	odom_msg.rpy.y = math.radians(0)
	odom_msg.rpy.z = math.radians(30)
	odom_msg.lin_vel.x = 0
	odom_msg.lin_vel.y = 0
	odom_msg.lin_vel.z = 0
	while not rospy.is_shutdown():
		rospy.loginfo("odom msg: lld[%s, %s, %s], rpy[%s, %s, %s], lin_vel[%s, %s, %s]" % (	odom_msg.lld.x,
													odom_msg.lld.y,
													odom_msg.lld.z,
													odom_msg.rpy.x,
													odom_msg.rpy.y,
													odom_msg.rpy.z,
													odom_msg.lin_vel.x,
													odom_msg.lin_vel.y,
													odom_msg.lin_vel.z))
		pub.publish(odom_msg)
		#odom_msg.lla.z = odom_msg.lla.z - 0.01
		#odom_msg.lld.x = odom_msg.lld.x + 0.000001
		rate.sleep()"""
	rospy.spin()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
