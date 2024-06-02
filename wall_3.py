#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

# Global variables for PID controller
previous_error = 0
integral = 0

# PID constants
kp = 0.5
ki = 0.0
kd = 0.0

# Distance thresholds
min_threshold = 5.0
max_threshold = 10.0

# Publisher for robot commands
pub_robot_commands = None

def sensor2_callback(data):
    global previous_error, integral, pub_robot_commands
    
    distance = data.data
    
    # Desired distance to the wall (middle of the threshold range)
    desired_distance = (min_threshold + max_threshold) / 2
    error = desired_distance - distance
    
    # PID control
    integral += error
    derivative = error - previous_error
    control = kp * error + ki * integral + kd * derivative
    previous_error = error
    
    # Determine command based on control value
    if abs(error) < min_threshold:
        command = 'w'  # Move forward
    elif control > 0:
        command = 'a'  # Turn left
    else:
        command = 'd'  # Turn right
    
    rospy.loginfo("Distance: %d, Error: %d, Control: %s, Command: %s" %(data.data , error , control,command))
    pub_robot_commands.publish(command)

def sensor_logic():
    global pub_robot_commands
    rospy.init_node('sensor_logic', anonymous=True)
    
    # Subscribe to distance_sensor1 topic
    rospy.Subscriber('distance_sensor2', Float32, sensor2_callback)
    
    # Publish robot commands
    pub_robot_commands = rospy.Publisher('/robot_commands', String, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        sensor_logic()
    except rospy.ROSInterruptException:
        pass

