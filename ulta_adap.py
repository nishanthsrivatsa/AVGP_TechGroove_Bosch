#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import serial

# Global variables
counter = 0
pub_robot_commands = None

# Define callback function for distance_sensor1 subscriber

        
        
def sensor2_callback(data):
    global counter, pub_robot_commands
    
    # Check if distance is between 20 and 100
    if data.data > 20:
        #counter += 20
        rospy.loginfo("Moving forward")
        pub_robot_commands.publish("f") # Move forward until next command 
    
    else:
        pub_robot_commands.publish("o")
        rospy.loginfo("STOP")

def sensor_logic():
    global pub_robot_commands
    rospy.init_node('sensor_logic', anonymous=True)
    
    # Subscribe to distance_sensor1 topic
    #rospy.Subscriber('distance_sensor1', Float32, sensor1_callback)
    rospy.Subscriber('distance_sensor2', Float32, sensor2_callback)

    # Initialize serial connection
    # ser = serial.Serial('/dev/ttyUSB0', 9600)  # Modify port and baudrate as needed
    
    # Publish robot commands
    pub_robot_commands = rospy.Publisher('/robot_commands', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    sensor_logic()

