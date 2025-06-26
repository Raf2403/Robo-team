#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

// Publishers declaration for each wheel
ros::Publisher front_left_wheel_pub, front_right_wheel_pub;
ros::Publisher back_left_wheel_pub, back_right_wheel_pub;

// Callback function to handle robot motion commands
void commandsCallback(const std_msgs::String::ConstPtr& msg) {
  // Initialize wheel speeds
  float leftSpeed = 0;
  float rightSpeed = 0;

  // Get command from the message
  std::string command = msg->data;

  // Handle different commands and set corresponding speeds
  if(command == "GO") 
  {
    leftSpeed = -1.0;
    rightSpeed = -1.0;
  }
  else if(command == "GO_REALLY_FAST")
  {
    leftSpeed = -10.0;
    rightSpeed = -10.0;
  }
  else if(command == "BACK") 
  {
    leftSpeed = 0.5;
    rightSpeed = 0.5;
  } 
  else if(command == "LEFT") 
  {
    leftSpeed = -1.0;
    rightSpeed = -0.5;
  } 
  else if(command == "RIGHT") 
  {
    leftSpeed = -0.5;
    rightSpeed = -1.0;
  } 
  else if(command == "STOP") 
  {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
  }

  // Form messages with set speeds
  std_msgs::Float64 msgFrontLeft, msgFrontRight;
  std_msgs::Float64 msgBackLeft, msgBackRight;

  msgFrontLeft.data = leftSpeed;
  msgBackLeft.data = leftSpeed;
  msgFrontRight.data = rightSpeed;
  msgBackRight.data = rightSpeed;

  // Publish speed messages for each wheel
  front_left_wheel_pub.publish(msgFrontLeft);
  back_left_wheel_pub.publish(msgBackLeft);
  front_right_wheel_pub.publish(msgFrontRight);
  back_right_wheel_pub.publish(msgBackRight);
}

int main(int argc, char **argv) {
  // Initialize ROS node and create subscribers and publishers for topics
  ros::init(argc, argv, "Simple_Controller");
  ros::NodeHandle n;

  // Create a subscriber to receive motion control commands
  ros::Subscriber commandSub = n.subscribe("my_motor_commands", 1000, commandsCallback);

  // Create publishers to control speeds of each wheel
  front_left_wheel_pub = n.advertise<std_msgs::Float64>("/front_left_wheel_controller/command", 1000);
  front_right_wheel_pub = n.advertise<std_msgs::Float64>("/front_right_wheel_controller/command", 1000);
  back_left_wheel_pub = n.advertise<std_msgs::Float64>("/back_left_wheel_controller/command", 1000);
  back_right_wheel_pub = n.advertise<std_msgs::Float64>("/back_right_wheel_controller/command", 1000);

  // Spin ROS events
  ros::spin();

  return 0;
}
