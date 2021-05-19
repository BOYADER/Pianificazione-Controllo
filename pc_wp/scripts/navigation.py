#!/usr/bin/env python
import rospy
from pc_wp.msg import Odom

def navigation():
	pub = rospy.Publisher('odom', Odom, queue_size=10)
	rospy.init_node('navigation')
	rate = rospy.Rate(1) # 1hz
	odom_msg = Odom()
	odom_msg.lla.x = 45.110735 
	odom_msg.lla.y = 7.640827 
	odom_msg.lla.z = 0
	odom_msg.rpy.x = 0
	odom_msg.rpy.y = 0
	odom_msg.rpy.z = 0
	odom_msg.lin_vel.x = 0
	odom_msg.lin_vel.y = 0
	odom_msg.lin_vel.z = 0
	while not rospy.is_shutdown():
		rospy.loginfo(odom_msg)
		pub.publish(odom_msg)
		#odom_msg.lla.z = odom_msg.lla.z - 0.01
		#odom_msg.lla.x = odom_msg.lla.x + 0.000001
		rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
