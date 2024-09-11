#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
using namespace std;

string m, m_old;       //mode
float angle = 0;
float speed = 0; 
//bool block_callbacks = 0;
int counter = 0;

void callback_mode(const std_msgs::String::ConstPtr& msg){
    m = msg->data;
    //printf("Mode in mux callback: %s \n", m.c_str());
    if(m_old == "p" && m == "g"){       //transitioning from pid to gamepad
        speed = 0;
        angle = 0;
        //block_callbacks = 1;
        //printf("Transition from pid to gamepad! \n");
    }
    m_old = m;
}


void callback_pid(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    if(m == "p"){
        angle = (msg->drive).steering_angle;
        speed = (msg->drive).speed;
        counter++;
        //printf("Counter %d \n", counter);
    }
}

void callback_keyboard(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    if(m == "k"){     
        angle = (msg->drive).steering_angle;
        speed = (msg->drive).speed;
    }
}

void callback_gamepad(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    if(m == "g"){     
        angle = (msg->drive).steering_angle;
        speed = (msg->drive).speed;
    }
}




int main(int argc, char ** argv){
    ros::init(argc, argv, "mux");
    ros::NodeHandle node;
    ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop", 1);
    ros::Subscriber keyboard_sub = node.subscribe("/drive_keyboard", 1, callback_keyboard);
    ros::Subscriber gamepad_sub = node.subscribe("/drive_gamepad", 1, callback_gamepad);
    ros::Subscriber follow_sub = node.subscribe("/drive_pid", 1, callback_pid);
    ros::Subscriber mode_sub = node.subscribe("/mode", 1, callback_mode);    

    ros::Rate loop_rate(50); 


    while(ros::ok()){
        //printf("Current mode selected as %s \n", m.c_str());
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
        command_pub.publish(drive_st_msg);      //do not publish commands in nthe drive topic during maneuver
        ros::spinOnce();            //pumping callbacks
        loop_rate.sleep();
    }
}
