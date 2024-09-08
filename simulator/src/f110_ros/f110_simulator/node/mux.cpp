#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
using namespace std;

string m;       //mode
string brake_state;
float angle = 0;
float speed = 0; 
bool brake_maneuver = 0; 
bool had_to_brake = 0;

void callback_mode(const std_msgs::String::ConstPtr& msg){
    if(!had_to_brake){        //after braking mode is fixed to keyboard
        m = msg->data;
    }
}

void callback_follow(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    if(m == "f"){
        angle = (msg->drive).steering_angle;
        speed = (msg->drive).speed;
    }
}

void callback_keyboard(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    if(m == "k"){     
        angle = (msg->drive).steering_angle;
        speed = (msg->drive).speed;
    }
}

void callback_brake(const std_msgs::String::ConstPtr& msg){
    brake_state = msg->data;
    if(brake_state == "m"){     //car maneuevering 
        m = "k";
        had_to_brake = 1;
        brake_maneuver = 1;
    }
    else if(brake_state == "o"){
        brake_maneuver = 0;
        speed = 0;
    } //maneuver just finished - stop the car

}






int main(int argc, char ** argv){
    ros::init(argc, argv, "mux");
    ros::NodeHandle node;
    ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
    ros::Subscriber keyboard_sub = node.subscribe("/drive_keyboard", 1, callback_keyboard);
    ros::Subscriber follow_sub = node.subscribe("/drive_follow", 1, callback_follow);
    ros::Subscriber mode_sub = node.subscribe("/mode", 1, callback_mode);    
    ros::Subscriber aeb_sub = node.subscribe("/brake", 1, callback_brake);

    ros::Rate loop_rate(50); 


    while(ros::ok()){
        // Make and publish message
        //  Header
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        //  Ackermann
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.speed = speed;
        drive_msg.steering_angle = angle;
        //  AckermannStamped
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        drive_st_msg.header = header;
        drive_st_msg.drive = drive_msg;
        // publish AckermannDriveStamped message to drive topic
        if(!brake_maneuver) command_pub.publish(drive_st_msg);      //do not publish commands in the drive topic during maneuver (AEB is publishing)
        ros::spinOnce();            //pumping callbacks
        loop_rate.sleep();
    }
}