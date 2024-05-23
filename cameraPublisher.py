#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('camera_publisher_node')
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue

        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(img_msg)

        rate.sleep()

    cap.release()
