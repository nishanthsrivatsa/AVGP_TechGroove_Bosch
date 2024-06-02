#include "ros.h"
#include <AFMotor.h>
#include "geometry_msgs/Twist.h"

AF_DCMotor motor1(1); // Motor 1 is connected to M1 port
AF_DCMotor motor2(2); // Motor 2 is connected to M2 port
AF_DCMotor motor3(3); // Motor 3 is connected to M3 port
AF_DCMotor motor4(4); // Motor 4 is connected to M4 port

float x,z; 

ros::NodeHandle nh;

void velCallback(  const geometry_msgs::Twist& vel)
{
     x = vel.linear.x ; // I CAN USE VEL AS I WANT
     z=vel.angular.z ;
     if (x >0) {
      motor1.setSpeed(255);
      motor1.run(FORWARD);
     }
     else if (x<0){
      motor1.setSpeed(255);
  motor1.run(BACKWARD);
     }
     else{
      motor1.setSpeed(0);
  motor1.run(RELEASE) ;
     }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

void setup() {
     nh.initNode();
     nh.subscribe(sub);
}

void loop() {
     nh.spinOnce();
     delay(10);
}
