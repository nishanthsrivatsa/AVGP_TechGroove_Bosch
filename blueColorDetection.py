#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import time

bridge = CvBridge()

blue_area_count = 0

cmd_pub = None

is_stopped = False

def publish_command(command):
    global cmd_pub
    cmd_pub.publish(command)
    rospy.loginfo(f"Published command: {command}")

def image_callback(data):
    global blue_area_count, is_stopped

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])

    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blue_area_count = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        
        if area > 500:  
            blue_area_count += 1
            
            cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 3)
            
            rospy.loginfo(f"Detected blue area {blue_area_count} with area: {area}")

            if blue_area_count == 1 and not is_stopped:
                publish_command('stop')
                is_stopped = True
                rospy.loginfo("Stopping for 10 seconds")
                time.sleep(10)
                is_stopped = False
                break

    cv2.imshow('Detected Blue Areas', cv_image)
    cv2.waitKey(1)

def main():
    global cmd_pub

    rospy.init_node('blue_detector', anonymous=True)
    
    cmd_pub = rospy.Publisher('/robot_commands', String, queue_size=10)
    
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    publish_command('w')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
