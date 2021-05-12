#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback(data):
	rospy.loginfo(data.data)
	
def model():
	rospy.init_node('model')
	rospy.Subscriber('tau', Float64, callback)
	rospy.spin()

if __name__ == '__main__':
	model()
