#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#define _USE_MATH_DEFINES
using namespace std;

int idx_max= 0;
int coll_idx = 0;           //collision index
int length_ranges = 0;
int length_cropped =0;
std::vector<float> ranges;
float yaw = 0;
float prev_yaw = 0;
float coll_angle = 0;
float a = 5; //crop factor
float phi_min = M_PI*(1/a-1);       //min and max angles corresponding to thwe cropped lidar array
float phi_max = M_PI*(1-1/a); 
float threshold = 0.75;
float K = 1.0;   
float fix_speed = 1.5; //max: 2.0
float max_speed = 4.0; 
float speed = 0;
float yaw_thresh = 1.0;


int max_element_index(std::vector<float> vec){  
    float max = vec[0];
    int max_idx = 0;
    for(int i = 0; i<vec.size(); i++){
       if(vec[i]>max){
        max = vec[i];
        max_idx = i;
       }
    }
    return max_idx;
}


float min_element(std::vector<float> vec){
    float min = vec[0];
    int min_idx = 0;
    for(int i = 0; i<vec.size(); i++){
       if(vec[i]<min){
        min = vec[i];
        min_idx = i;
       }
    }
    return min;
}


int min_element_idx(std::vector<float> vec){
    float min = vec[0];
    int min_idx = 0;
    for(int i = 0; i<vec.size(); i++){
       if(vec[i]<min){
        min = vec[i];
        min_idx = i;
       }
    }
    return min_idx;
}

 

void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    ranges = msg->ranges;
    length_ranges = ranges.size();

    //Cropping to exclude back of the car
    std::vector<float> cropped_ranges(ranges.begin() + length_ranges/a, ranges.end() - length_ranges/a);
    //Finding the farthest region 
    idx_max = max_element_index(cropped_ranges);
    //printf("Max index is: %d/n", idx_max);
    length_cropped = cropped_ranges.size();
    yaw = (2*M_PI*(1-1/a)/(length_cropped-1))*idx_max+phi_min; //cropped array
    if(min_element(cropped_ranges)<threshold){      //check for collisions
        coll_idx = min_element_idx(cropped_ranges);
        coll_angle = (2*M_PI*(1-1/a)/(length_cropped-1))*coll_idx+phi_min;
        yaw = -K*coll_angle;
    }
    //Dynamic speed adjustment
    //speed = (abs(yaw)>yaw_thresh)?0.5:max_speed;
    //Fixed speed
    speed = fix_speed;
}



int main(int argc, char ** argv){
    ros::init(argc, argv, "follow_gap");
    ros::NodeHandle node;
    //ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);        //for running the follow_gap alone
    ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_follow", 1);  //mux integration
    ros::Subscriber lidar_sub = node.subscribe("/scan", 1, callback);
    ros::Rate loop_rate(50);   
    ackermann_msgs::AckermannDrive drive_msg;
    while(ros::ok()){
        drive_msg.steering_angle = yaw;
        drive_msg.speed = speed;

        std_msgs::Header header;

        //  AckermannStamped
        header.stamp = ros::Time::now();
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        drive_st_msg.header = header;
        drive_st_msg.drive = drive_msg;
        command_pub.publish(drive_st_msg);
        ros::spinOnce();        //pumping subscriber callbacks 
        loop_rate.sleep();
    }
    return 0;

}