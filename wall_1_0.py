#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

# PID parameters
Kp = 1.0
Ki = 0.1
Kd = 0.05

# Global variables
counter = 0
pub_robot_commands = None
desired_distance = 6.5  # Desired distance from the wall (between 5 and 8)
previous_error = 0
integral = 0

# Define callback function for distance_sensor1 subscriber
def sensor1_callback(data):
    global counter, pub_robot_commands, previous_error, integral
    
    # Compute error
    error = desired_distance - data.data
    
    # Compute PID components
    P = Kp * error
    integral += error
    I = Ki * integral
    D = Kd * (error - previous_error)
    
    # Compute PID output
    output = P + I + D
    previous_error = error
    
    # Determine robot command based on PID output
    if abs(output) < 0.1:
        command = "w"  # Move forward if no significant correction is needed
    elif output > 0:
        command = "a"  # Move left if too close to the wall
    else:
        command = "d"  # Move right if too far from the wall
    
    # Publish command
    pub_robot_commands.publish(command)

def sensor_logic():
    global pub_robot_commands
    rospy.init_node('sensor_logic', anonymous=True)
    
    # Subscribe to distance_sensor1 topic
    rospy.Subscriber('distance_sensor1', Float32, sensor1_callback)

    # Publish robot commands
    pub_robot_commands = rospy.Publisher('/robot_commands', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    sensor_logic()
