import rospy
from std_msgs.msg import Float32, String
import serial

# Global variables
pub_robot_commands = None
desired_distance = 20  # Desired distance between the bots in cm
prev_error = 0  # Previous error for integral term
dict_constants = {(0,20):[2,0.5],(20,40):[4,1],(40,60):[6,2]}

def get_values(error, data):
    for key in data:
        if key[0] <= error <= key[1]:
            Kp, Ki = data[key]
            return Kp, Ki
    return None
#print(get_values(30,dict_constants))

# Define callback function for distance_sensor1 subscriber

def sensor1_callback(data):
    global pub_robot_commands, prev_error
    
    # Calculate error (difference between desired distance and actual distance)
    error = desired_distance - data.data
    
    # Integral term
    integral = prev_error + error
    Kp = get_values(error, dict_constants)[0]
    Ki = get_values(error, dict_constants)[1]
    # Calculate speed based on proportional and integral control
    speed = int(Kp * error + Ki * integral)
    speed = min(max(speed, 0), 255)  # Limit speed to range [0, 255]
    
    # Publish speed command
    pub_robot_commands.publish(str(speed))
    
    # Update previous error
    prev_error = error

def sensor_logic():
    global pub_robot_commands
    rospy.init_node('sensor_logic', anonymous=True)
    
    # Subscribe to distance_sensor1 topic
    rospy.Subscriber('distance_sensor2', Float32, sensor1_callback)

    # Initialize serial connection
    # ser = serial.Serial('/dev/ttyUSB0', 9600)  # Modify port and baudrate as needed
    
    # Publish robot speed commands
    pub_robot_commands = rospy.Publisher('/robot_speed', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    sensor_logic()
