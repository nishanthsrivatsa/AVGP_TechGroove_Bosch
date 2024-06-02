#!/usr/bin/env python3

import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import Range

# Global variables to store sensor readings
right_distance = 0

def wall_following():
    global right_distance
    
    # Ensure valid sensor readings
    if right_distance <= 0:
        rospy.logwarn("Invalid sensor reading. Skipping control loop.")
        return
    
    # Desired distance from the wall
    desired_right_distance = 0.10  # 10 cm
    
    # Error is the difference between desired distance and actual distance
    error_right = desired_right_distance - right_distance
    
    # Control logic based on error
    if abs(error_right) < 0.01:  # If the error is small, maintain current position
        command = 'w'
    elif error_right > 0:  # If the robot is too close to the wall, move right
        command = 'd'
    else:  # If the robot is too far from the wall, move left
        command = 'a'
    
    # Publish command
    command_pub.publish(command)

def right_distance_callback(msg):
    global right_distance
    right_distance = msg.range
    wall_following()

def shutdown_hook():
    # Publish stop command before shutting down
    command_pub.publish('s')

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    
    # Initialize publishers and subscribers
    rospy.Subscriber('/right_ultrasonic_sensor', Range, right_distance_callback)
    command_pub = rospy.Publisher('/robot_commands', Char, queue_size=10)
    
    # Register shutdown hook
    rospy.on_shutdown(shutdown_hook)
    
    # Set the loop rate for the control loop
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        wall_following()
        rate.sleep()
