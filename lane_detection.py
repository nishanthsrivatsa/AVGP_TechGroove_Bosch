#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

bridge = CvBridge()
pub = None

def process_frame(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    blur = cv2.GaussianBlur(binary, (5, 5), 0)

    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height * 0.6)),
        (0, int(height * 0.6))
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)

    line_image = np.zeros_like(frame)

    left_line_x = []
    right_line_x = []

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 < width // 2 and x2 < width // 2:
                    left_line_x.append(x1)
                    left_line_x.append(x2)
                elif x1 > width // 2 and x2 > width // 2:
                    right_line_x.append(x1)
                    right_line_x.append(x2)
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)

    combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return combo_image, left_line_x, right_line_x

def determine_command(left_line_x, right_line_x, width):
    command = String()
    if len(left_line_x) > 0 and len(right_line_x) > 0:
        left_mean = np.mean(left_line_x)
        right_mean = np.mean(right_line_x)
        lane_center = (left_mean + right_mean) / 2
        frame_center = width / 2

        if lane_center < frame_center - 50:
            command.data = "L"
        elif lane_center > frame_center + 50:
            command.data = "R"
        else:
            command.data = "F"
    else:
        command.data = "S"

    return command

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    lane_frame, left_line_x, right_line_x = process_frame(frame)
    cv2.imshow("Lane Detection", lane_frame)
    cv2.waitKey(1)

    height, width, _ = frame.shape
    command = determine_command(left_line_x, right_line_x, width)
    pub.publish(command)

def main():
    global pub
    rospy.init_node('lane_detection_node')
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    pub = rospy.Publisher('motor_commands', String, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
