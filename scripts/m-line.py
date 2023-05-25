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

def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_angle(p1, p2):
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

def move_to_goal(goal_x, goal_y):
    target_x = goal_x
    target_y = goal_y

    min_distance = 0.1
    buffer_distance = 0.5

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(5) # 10hz
    velocity = Twist()
    obstacle_angle = 0.0
    obstacle_distance = float('inf')

    while not rospy.is_shutdown():
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y

        distance_to_goal = calculate_distance((x, y), (target_x, target_y))
        angle_to_goal = calculate_angle((x, y), (target_x, target_y))

        if distance_to_goal > min_distance:
            if obstacle_distance > buffer_distance:
                velocity.linear.x = 0.5
                velocity.angular.z = 0.0
            else:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.3

            pub.publish(velocity)

            if obstacle_distance <= buffer_distance:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.3
                pub.publish(velocity)

        else:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            pub.publish(velocity)
            rospy.loginfo("Goal reached!")
            break

        r.sleep()

if __name__ == "__main__":
    rospy.init_node("stage_controller_node", anonymous=False)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)
    rospy.Subscriber("/base_scan", LaserScan, laser_callback)

    goal_x = 8.0
    goal_y = 13.0

    move_to_goal(goal_x, goal_y)


if laser.ranges: obstacle_detected = min(laser.ranges) < min_distance