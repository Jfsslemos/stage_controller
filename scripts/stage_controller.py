#! /usr/bin/env python
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import random
import math
import numpy as np

laser = LaserScan()
odometry = Odometry()

def odometry_callback(data):
	global odometry
	odometry = data
	
def laser_callback(data):
	global laser
	laser = data

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def calculate_slope(p1, p2):
    # Calculate the slope between two points
    if p2[0] - p1[0] != 0:
        slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
        return slope
    else:
        # Handle the case when the line is vertical
        return float('inf')

def calculate_intercept(p, slope):
    # Calculate the y-intercept of the line
    if slope != float('inf'):
        intercept = p[1] - slope * p[0]
        return intercept
    else:
        # Handle the case when the line is vertical
        return p[0]

def is_point_on_line(p, p1, p2, min_distance):
    # Check if a point is on the line defined by two other points,
    # considering a minimal distance threshold
    slope = calculate_slope(p1, p2)
    intercept = calculate_intercept(p1, slope)

    if slope != float('inf'):
        expected_y = slope * p[0] + intercept
        distance = abs(p[1] - expected_y)
    else:
        # Handle the case when the line is vertical
        distance = abs(p[0] - p1[0])

    return distance <= min_distance

def deviation_estimate(target_x, target_y, x, y):

	deviation_x = target_x -x 
	deviation_y = target_y - y
	return math.atan2(deviation_y, deviation_x)


if __name__ == "__main__": 
	rospy.init_node("stage_controller_node", anonymous=False)  

	rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

	rospy.Subscriber("/base_scan", LaserScan, laser_callback)

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  

	r = rospy.Rate(5) # 10hz

	velocity = Twist()

	target_x = 1.0
	target_y = 1.0
	min_distance = 0.5

	while not rospy.is_shutdown():

		x = odometry.pose.pose.position.x
		y = odometry.pose.pose.position.y

		distance = math.sqrt((x-target_x)**2 + (y-target_y)**2)
                
		# laser.ranges[901]
		# laser.ranges[180]

		# Verifica se chegou ao alvo
		if (distance > 0.2):
			if laser.ranges:
				nearest_obstacle = min(laser.ranges)
				if nearest_obstacle < 0.2:
					velocity.angular.z = 0.0
					velocity.linear.x = -0.5
				else:
					roll, pitch, yaw = euler_from_quaternion(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
					angle = deviation_estimate(target_x, target_y, x, y)
					if angle - yaw > 0.1:
						velocity.angular.z = 0.4
						velocity.linear.x = 0.0
						pub.publish(velocity)
					elif angle - yaw < -0.1:
						velocity.angular.z = -0.4
						velocity.linear.x = 0.0
						pub.publish(velocity)
					else:
						velocity.linear.x = 1.0
						velocity.angular.z = 0.0
						pub.publish(velocity)

		else:
			velocity.linear.x = 0.0
			velocity.angular.z = 0.0
			pub.publish(velocity)
			rospy.loginfo("Alvo Alcancado!!!!!")		 

		# r.sleep()