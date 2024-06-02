#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Initialize the ROS node
rospy.init_node('blue_lane_detector', anonymous=True)

# Publisher for motor commands
motor_command_pub = rospy.Publisher('/robot_commands', String, queue_size=10)

# Initialize the CvBridge
bridge = CvBridge()

# Blue color range in HSV
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

blue_count = 0

def detect_blue(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_detected = np.any(mask)
    return blue_detected

def image_callback(msg):
    global blue_count
    try:
        # Convert the ROS Image message to OpenCV2 format
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        if detect_blue(cv2_img):
            blue_count += 1
            rospy.loginfo(f"Blue detected! Count: {blue_count}")
            
            if blue_count == 2:
                rospy.loginfo("Count is 2. Stopping for 10 seconds...")
                motor_command_pub.publish('s')  # Publish stop command
                rospy.sleep(10)
                blue_count = 0  # Reset counter after stopping
            else:
                motor_command_pub.publish('w')  # Publish move forward command
        else:
            motor_command_pub.publish('w')  # Publish move forward command

# Subscriber for image messages
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Keep the program running
rospy.spin()
