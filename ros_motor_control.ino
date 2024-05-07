
#include <AFMotor.h>
#include <ros.h>
#include <std_msgs/String.h>

AF_DCMotor motor1(1); // Motor 1 is connected to M1 port
AF_DCMotor motor2(2); // Motor 2 is connected to M2 port
AF_DCMotor motor3(3); // Motor 3 is connected to M3 port
AF_DCMotor motor4(4); // Motor 4 is connected to M4 port

ros::NodeHandle nh;

void commandCallback(const std_msgs::String& msg) {
  char command = msg.data[0];

  switch (command) {
    case 'w': // Move forward
      moveForward();
      
      break;
    case 's': // Move backward
      moveBackward;
      
      break;
    case 'a': // Turn left
      turnLeft();
      break;
    case 'd': // Turn right
      turnRight();
      break;
    case 'o': // Stop
      stopMotors();
      break;
  }
}

ros::Subscriber<std_msgs::String> sub("/robot_commands", &commandCallback);

void setup() {
  Serial.begin(9600); // Initialize serial communication
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}

void moveForward() {
  Serial.print("Forward command Recived");
  motor1.setSpeed(255);
  motor1.run(FORWARD);
  motor2.setSpeed(255);
  motor2.run(FORWARD);
  motor3.setSpeed(255);
  motor3.run(FORWARD);
  motor4.setSpeed(255);
  motor4.run(FORWARD);
}

void moveBackward() {
  Serial.print("backward command Recived");
  motor1.setSpeed(255);
  motor1.run(BACKWARD);
  motor2.setSpeed(255);
  motor2.run(BACKWARD);
  motor3.setSpeed(255);
  motor3.run(BACKWARD);
  motor4.setSpeed(255);
  motor4.run(BACKWARD);
}

void turnLeft() {
  motor1.setSpeed(150);
  motor1.run(FORWARD);
  motor2.setSpeed(150);
  motor2.run(FORWARD);
  motor3.setSpeed(150);
  motor3.run(BACKWARD);
  motor4.setSpeed(150);
  motor4.run(BACKWARD);
}

void turnRight() {
  motor1.setSpeed(150);
  motor1.run(BACKWARD);
  motor2.setSpeed(150);
  motor2.run(BACKWARD);
  motor3.setSpeed(150);
  motor3.run(FORWARD);
  motor4.setSpeed(150);
  motor4.run(FORWARD);
}

void stopMotors() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);
}
void park() {
  moveForward();
  delay(100);
  turnLeft();
  delay(50);
  stopMotors();
  delay(10);
  turnRight();
  delay(50);
  stopMotors();
  delay(10000000);
    
}
