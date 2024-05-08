import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

desired_distance = rospy.get_param('~desired_distance', 1.5)
max_speed = rospy.get_param('~max_speed', 5.0)
min_speed = rospy.get_param('~min_speed', 0.0)
acceleration = rospy.get_param('~acceleration', 0.1)
brake_distance = rospy.get_param('~brake_distance', 0.5)

distance_to_vehicle = None

def distance_callback(data):
    global distance_to_vehicle
    distance_to_vehicle = data.range

def control_loop():
    rospy.init_node('Adaptive_cruise_control')
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('distance_to_vehicle', Range, distance_callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if distance_to_vehicle is not None:
            twist_msg = Twist()
            if distance_to_vehicle > desired_distance:
                twist_msg.linear.x += min(max_speed, twist_msg.linear.x + acceleration)
            elif distance_to_vehicle < brake_distance:
                twist_msg.linear.x = max(min_speed, twist_msg.linear.x - acceleration)

            cmd_pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    control_loop()
    
