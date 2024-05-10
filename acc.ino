#include <AFMotor.h>
#include <ros.h>
#include <std_msgs/Int16.h> // Include the message type for speed command

AF_DCMotor motor1(1); // Motor 1 is connected to M1 port
AF_DCMotor motor2(2); // Motor 2 is connected to M2 port
AF_DCMotor motor3(3); // Motor 3 is connected to M3 port
AF_DCMotor motor4(4); // Motor 4 is connected to M4 port

ros::NodeHandle nh;
ros::Publisher string_data("message", &msg);
std_msgs::String msg;

void commandCallback(const std_msgs::Int16& msg) {
  int speed = msg.data;
  moveForward(speed);
}

ros::Subscriber<std_msgs::Int16> sub("/robot_speed", &commandCallback);

void setup() {
  Serial.begin(9600); // Initialize serial communication
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(string_data);
}

void loop() {
  msg.data = "Hello";
  string_data.publish(&msg);
  nh.spinOnce();
}

void moveForward(int speed) {
  motor1.setSpeed(speed);
  motor1.run(FORWARD);
  motor2.setSpeed(speed);
  motor2.run(FORWARD);
  motor3.setSpeed(speed);
  motor3.run(FORWARD);
  motor4.setSpeed(speed);
  motor4.run(FORWARD);

}
