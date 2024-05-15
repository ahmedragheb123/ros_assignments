// Include necessary libraries for the Arduino board
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Motor pins
const int LEFT_PWM_PIN = 5; // PWM pin for left motor speed control
const int LEFT_INT1_PIN = 6; // Control pin 1 for left motor
const int LEFT_INT2_PIN = 7; // Control pin 2 for left motor
const int RIGHT_PWM_PIN = 9; // PWM pin for right motor speed control
const int RIGHT_INT1_PIN = 8; // Control pin 1 for right motor
const int RIGHT_INT2_PIN = 4; // Control pin 2 for right motor


// Initialize ROS node handle
ros::NodeHandle nh;


// Initialize ROS publisher
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {

  int pwm_linear = map((long) cmd_vel.linear.x, 0, 1, 0, 80);
  int pwm_angular = map((long) cmd_vel.angular.z, 0, 1, 0, 80);

  if(pwm_linear == 0){
  digitalWrite(RIGHT_INT1_PIN,HIGH);
  digitalWrite(RIGHT_INT2_PIN, LOW);
  digitalWrite(LEFT_INT1_PIN, LOW);
  digitalWrite(LEFT_INT2_PIN, HIGH);
  analogWrite(LEFT_PWM_PIN, pwm_angular);
  analogWrite(RIGHT_PWM_PIN, pwm_angular);
  }
  else{
  digitalWrite(RIGHT_INT1_PIN, HIGH);
  digitalWrite(RIGHT_INT2_PIN, LOW);
  digitalWrite(LEFT_INT1_PIN, HIGH);
  digitalWrite(LEFT_INT2_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, pwm_linear);
  analogWrite(RIGHT_PWM_PIN, pwm_linear);
  }

}
    
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdVelCallback);

void setup(){

  nh.getHardware()->setBaud(57600);
  // Initialize ROS node
  nh.initNode();
  nh.subscribe(cmd_sub);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_INT1_PIN, OUTPUT);
  pinMode(LEFT_INT2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_INT1_PIN, OUTPUT);
  pinMode(RIGHT_INT2_PIN, OUTPUT);
}

void loop(){

  // Spin ROS node
  nh.spinOnce();

}