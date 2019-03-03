#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('tunnel', anonymous=True)

pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)

def turtlestop():

	twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

def turtlemove(linear,angular):

	rospy.on_shutdown(turtlestop)
	#print(angular)
	twist=Twist()
	twist.linear.x=linear
	twist.angular.z=angular
	pub.publish(twist)

def linearmove(cm):
	
	for i in range(cm+1):

		turtlemove(0.05,0)
		rospy.sleep(rospy.Duration(0.3))
		turtlemove(0,0)
		rospy.sleep(rospy.Duration(0.3))


def angularmove(degree): #right + left -

	abs_degree = abs(degree)

	if degree>0:

		for i in range(abs_degree):

			turtlemove(0,-0.1)
			rospy.sleep(rospy.Duration(0.2))
			turtlemove(0,0)
	else:

		for i in range(abs_degree):

			turtlemove(0,0.1)
			rospy.sleep(rospy.Duration(0.2))
			turtlemove(0,0)

def callback(lidar):

	d=lidar.ranges

if __name__ == '__main__':
	print('d')
	linearmove(10)


rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
