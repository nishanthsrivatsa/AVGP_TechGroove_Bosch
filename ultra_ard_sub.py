#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import serial

# Global variables
counter = 0
pub_robot_commands = None

# Define callback function for distance_sensor1 subscriber
def sensor1_callback(data):
    global counter, pub_robot_commands
    
    # Check if distance is between 20 and 100
    if 15 < data.data < 60:
        counter += 1
        rospy.loginfo("Counter: %d" % counter)
        pub_robot_commands.publish("f") # Move forward until next command 
    else:
        # Check counter value
        if counter > 5:
            rospy.loginfo("Ready to park")
            # Send command to robot for parking
            pub_robot_commands.publish("P")  # Assuming 'P' is the command for parking
            counter=0
        else:
            rospy.loginfo("Counter reset")
            counter = 0
            pub_robot_commands.publish("o")

def sensor_logic():
    global pub_robot_commands
    rospy.init_node('sensor_logic', anonymous=True)
    
    # Subscribe to distance_sensor1 topic
    rospy.Subscriber('distance_sensor1', Float32, sensor1_callback)

    # Initialize serial connection
    # ser = serial.Serial('/dev/ttyUSB0', 9600)  # Modify port and baudrate as needed
    
    # Publish robot commands
    pub_robot_commands = rospy.Publisher('/robot_commands', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    sensor_logic()
