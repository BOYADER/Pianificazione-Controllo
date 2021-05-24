#!/usr/bin/env python
import rospy
import math
from pc_wp.msg import Odom

QUEUE_SIZE = rospy.get_param('/QUEUE_SIZE')

def navigation():
	pub = rospy.Publisher('odom', Odom, queue_size = QUEUE_SIZE)
	rospy.init_node('navigation')
	rate = rospy.Rate(1) # 1hz
	odom_msg = Odom()
	odom_msg.lld.x = 45.110735 
	odom_msg.lld.y = 7.640827 
	odom_msg.lld.z = 0
	odom_msg.rpy.x = math.radians(0)
	odom_msg.rpy.y = math.radians(10)
	odom_msg.rpy.z = math.radians(30)
	odom_msg.lin_vel.x = 0
	odom_msg.lin_vel.y = 0
	odom_msg.lin_vel.z = 0
	while not rospy.is_shutdown():
		print(odom_msg)
		pub.publish(odom_msg)
		#odom_msg.lla.z = odom_msg.lla.z - 0.01
		#odom_msg.lld.x = odom_msg.lld.x + 0.000001
		rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
