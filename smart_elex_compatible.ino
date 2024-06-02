#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

// Define motor pins
const int DIR1_PIN = 2;  // Direction pin for Motor 1
const int PWM1_PIN = 3;  // PWM pin for Motor 1
const int DIR2_PIN = 4;  // Direction pin for Motor 2
const int PWM2_PIN = 5;  // PWM pin for Motor 2


const int fbspeed =255;
const int turn_speed =130;
void commandCallback(const std_msgs::String& msg) {
  char command = msg.data[0];

  switch (command) {
    case 'w': // Move forward
      moveBackward(fbspeed);
      break;
    case 's': // Move backward
    moveForward(fbspeed);
      
      break;
    case 'a': // Turn left
      turnLeft(turn_speed);
      break;
    case 'd': // Turn right
      turnRight(turn_speed);
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

  // Initialize the motor control pins as outputs
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  // Set initial states
  digitalWrite(DIR1_PIN, LOW);
  analogWrite(PWM1_PIN, 0);
  digitalWrite(DIR2_PIN, LOW);
  analogWrite(PWM2_PIN, 0);

  // Subscribe to the ROS topic for commands
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}




void moveForward(int fbspeed) {
  digitalWrite(DIR1_PIN, HIGH);
  analogWrite(PWM1_PIN, fbspeed-4);
  digitalWrite(DIR2_PIN, HIGH);
  analogWrite(PWM2_PIN, fbspeed);
}

void moveBackward(int fbspeed) {
  digitalWrite(DIR1_PIN, LOW);
  analogWrite(PWM1_PIN, fbspeed-4);
  digitalWrite(DIR2_PIN, LOW);
  analogWrite(PWM2_PIN, fbspeed);
}

void turnLeft(int turn_speed) {
  digitalWrite(DIR2_PIN, HIGH);
  analogWrite(PWM1_PIN,turn_speed);
  digitalWrite(DIR1_PIN, LOW);
  analogWrite(PWM2_PIN, turn_speed);
}

void turnRight( int turn_speed) {
  digitalWrite(DIR2_PIN, LOW);
  analogWrite(PWM1_PIN, turn_speed);
  digitalWrite(DIR1_PIN, HIGH);
  analogWrite(PWM2_PIN, turn_speed);
}

void stopMotors() {
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
}
