#! /usr/bin/env python
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import random
import math

laser = LaserScan()
odometry = Odometry()

def odometry_callback(data):
	global odometry
	odometry = data
	
def laser_callback(data):
	global laser
	laser = data

if __name__ == "__main__": 
	rospy.init_node("stage_controller_node", anonymous=False)  

	rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

	rospy.Subscriber("/base_scan", LaserScan, laser_callback)

	target_x = 8.0
	target_y = 13.0

	min_distance = 0.1

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
	r = rospy.Rate(5) # 10hz
	velocity = Twist()
	while not rospy.is_shutdown():
		#print(laser.ranges)
		x = odometry.pose.pose.position.x
		y = odometry.pose.pose.position.y
		
		# Verifica se chegou ao alvo
		distance = math.sqrt((x-target_x)**2 + (y-target_y)**2)
		if (distance > min_distance):
			if (laser.ranges):
				obstacle = min(laser.ranges)
				if obstacle > 0.2:
					velocity.linear.x = 0.3
					velocity.angular.z = 0.0
					pub.publish(velocity)
				else:
					if laser.ranges.index(obstacle) < 540:
						velocity.linear.x = -0.1
						velocity.angular.z = 0.3
						pub.publish(velocity)
					
					elif laser.ranges.index(obstacle) > 540:
						velocity.linear.x = -0.1
						velocity.angular.z = -0.3
						pub.publish(velocity)
					else:
						velocity.linear.x = -0.1	
						velocity.angular.z = 0.2
						pub.publish(velocity)

		else:
			velocity.linear.x = 0.0
			velocity.angular.z = 0.0
			pub.publish(velocity)
			rospy.loginfo("Alvo Alcancado!!!!!")		 
		
		# r.sleep()
		



