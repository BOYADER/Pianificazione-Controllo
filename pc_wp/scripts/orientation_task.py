#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback(data):
	rospy.loginfo(data.data)
	pub = rospy.Publisher('references', Float64, queue_size = 10)
	pub.publish(data.data + 0.5)
	

def orientation_task():
	rospy.init_node('orientaton_task')
	rospy.Subscriber('current_task', Float64, callback)
	rospy.Subscriber('odom', Float64, callback)
	rospy.spin()

if __name__ == '__main__':
	orientation_task()
	
