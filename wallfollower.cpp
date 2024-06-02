#include "ros.h"
#include <AFMotor.h>
#include "geometry_msgs/Twist.h"

// Initialize the motors
AF_DCMotor motor1(1); // Motor 1 is connected to M1 port
AF_DCMotor motor2(2); // Motor 2 is connected to M2 port
AF_DCMotor motor3(3); // Motor 3 is connected to M3 port
AF_DCMotor motor4(4); // Motor 4 is connected to M4 port

float linear_x = 0;
float angular_z = 0;

ros::NodeHandle nh;

// Velocity callback function
void velCallback(const geometry_msgs::Twist& vel) {
    linear_x = vel.linear.x;
    angular_z = vel.angular.z;

    int speed_linear = 255 * fabs(linear_x);  // Scale linear speed
    int speed_angular = 255 * fabs(angular_z); // Scale angular speed

    // Forward or backward movement
    if (linear_x > 0) {
        motor1.setSpeed(speed_linear);
        motor1.run(FORWARD);
        motor2.setSpeed(speed_linear);
        motor2.run(FORWARD);
        motor3.setSpeed(speed_linear);
        motor3.run(FORWARD);
        motor4.setSpeed(speed_linear);
        motor4.run(FORWARD);
    } else if (linear_x < 0) {
        motor1.setSpeed(speed_linear);
        motor1.run(BACKWARD);
        motor2.setSpeed(speed_linear);
        motor2.run(BACKWARD);
        motor3.setSpeed(speed_linear);
        motor3.run(BACKWARD);
        motor4.setSpeed(speed_linear);
        motor4.run(BACKWARD);
    } else if (angular_z > 0) { // Turn left
        motor1.setSpeed(speed_angular);
        motor1.run(BACKWARD);
        motor2.setSpeed(speed_angular);
        motor2.run(FORWARD);
        motor3.setSpeed(speed_angular);
        motor3.run(BACKWARD);
        motor4.setSpeed(speed_angular);
        motor4.run(FORWARD);
    } else if (angular_z < 0) { // Turn right
        motor1.setSpeed(speed_angular);
        motor1.run(FORWARD);
        motor2.setSpeed(speed_angular);
        motor2.run(BACKWARD);
        motor3.setSpeed(speed_angular);
        motor3.run(FORWARD);
        motor4.setSpeed(speed_angular);
        motor4.run(BACKWARD);
    } else { // Stop
        motor1.setSpeed(0);
        motor1.run(RELEASE);
        motor2.setSpeed(0);
        motor2.run(RELEASE);
        motor3.setSpeed(0);
        motor3.run(RELEASE);
        motor4.setSpeed(0);
        motor4.run(RELEASE);
    }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
    delay(10);
}
