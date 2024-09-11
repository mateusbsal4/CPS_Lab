
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

bool publish_mode = 0;
ackermann_msgs::AckermannDrive drive_msg;

void callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    drive_msg = msg->drive;
    if(drive_msg.speed != 0 || drive_msg.steering_angle != 0){
          publish_mode = 1;
    }
}

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "gamepad");

  ros::NodeHandle n;
  ros::Publisher mode_pub = n.advertise<std_msgs::String>("/mode", 1);
  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_gamepad", 1);
  ros::Subscriber gamepad_sub = n.subscribe("/vesc/js", 1, callback); 

  ros::Rate loop_rate(50);  
  std::string mode = "g";
  while(ros::ok()) {
    publish_mode = 0;
    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);
    ros::spinOnce();        //pumping subscriber callbacks 
    if(publish_mode){
      std_msgs::String mode_msg = stdStringToRosString(mode);
      mode_pub.publish(mode_msg);
      //printf("Switching to gamepad mode \n");
    }
    loop_rate.sleep();
  }
  return 0;
}
