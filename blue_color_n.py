#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

rospy.init_node('blue_color_detection')

bridge = CvBridge()
pub = rospy.Publisher('/robot_commands', String, queue_size=10)  # Publisher for robot commands

counter = 0  # Initialize counter

def detect_blue_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([140, 255, 255])
    
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        x, y, w, h = cv2.boundingRect(contours[0])
        
        if y > frame.shape[0] * 0.75:
            return True
    
    return False

def image_callback(msg):
    global counter  # Use global keyword to modify counter inside the function
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    if detect_blue_color(frame):
        counter += 1
        if counter > 2:
            data = 'o'  # Stop command
        else:
            data = 'w'  # Move forward command
    else:
        counter = 0  # Reset counter if no blue color detected
        data = 's'  # Stop command
    
    # Publish motor command
    pub.publish(data)

# Subscribe to the camera image topic
rospy.Subscriber('/camera/image', Image, image_callback)
# Spin ROS
rospy.spin()
