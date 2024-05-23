#include <ros.h>
#include <std_msgs/String.h>

const int leftMotorForward = 9;
const int leftMotorBackward = 10;
const int rightMotorForward = 11;
const int rightMotorBackward = 12;

ros::NodeHandle nh;

void messageCb(const std_msgs::String& msg) {
    if (msg.data == "F") {
        moveForward();
    } else if (msg.data == "B") {
        moveBackward();
    } else if (msg.data == "L") {
        turnLeft();
    } else if (msg.data == "R") {
        turnRight();
    } else if (msg.data == "S") {
        stopMotors();
    }
}

ros::Subscriber<std_msgs::String> sub("motor_commands", &messageCb);

void setup() {
    pinMode(leftMotorForward, OUTPUT);
    pinMode(leftMotorBackward, OUTPUT);
    pinMode(rightMotorForward, OUTPUT);
    pinMode(rightMotorBackward, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
}

void moveForward() {
    digitalWrite(leftMotorForward, HIGH);
    digitalWrite(leftMotorBackward, LOW);
    digitalWrite(rightMotorForward, HIGH);
    digitalWrite(rightMotorBackward, LOW);
}

void moveBackward() {
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBackward, HIGH);
    digitalWrite(rightMotorForward, LOW);
    digitalWrite(rightMotorBackward, HIGH);
}

void turnLeft() {
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBackward, HIGH);
    digitalWrite(rightMotorForward, HIGH);
    digitalWrite(rightMotorBackward, LOW);
}

void turnRight() {
    digitalWrite(leftMotorForward, HIGH);
    digitalWrite(leftMotorBackward, LOW);
    digitalWrite(rightMotorForward, LOW);
    digitalWrite(rightMotorBackward, HIGH);
}

void stopMotors() {
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBackward, LOW);
    digitalWrite(rightMotorForward, LOW);
    digitalWrite(rightMotorBackward, LOW);
}
