#!/usr/bin/env python3

import rospy
from std_msgs.msg import String



def ros_publisher():
    rospy.init_node('ros_string_publisher', anonymous=True)
    pub = rospy.Publisher('/string_command', String, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        speed =255
        command = str(speed)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        ros_publisher()
    except rospy.ROSInterruptException:
        pass
