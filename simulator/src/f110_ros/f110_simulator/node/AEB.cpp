#include <ros/ros.h>
#include <string>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/String.h>
//#include <nav_msgs/Odometry.h>
using namespace std;

float v = 0;
float ttc = 0;
string m;
float threshold = 0.5; //seconds 
int L = 0;      //length of the uncropped ranges vector
float alpha = 0;            //angle
bool correct_pose = 0;
float collision_v =0;
//float collision_alpha = 0;
std::vector<float> ranges;



void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    correct_pose = 0;            //only publish msgs if braking condition is satisfied
    ranges = msg->ranges;
    L = ranges.size();
    for(int i = 0; i< L; i++){
        alpha = ((2*M_PI)/(L-1))*i-M_PI; 
        //printf("alpha: %f\n", alpha);
        ttc = abs(ranges[i]/(v*cos(alpha)));
       // printf("TTC: %f\n", ttc);
        if(ttc<threshold){
            collision_v = v;
            //printf("Brake order detected!!\n");
            correct_pose = 1;
            break;
        }
    }
}

void callback_drive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
    v = (msg->drive).speed;
}

void callback_mode(const std_msgs::String::ConstPtr& msg){
    m = msg->data;
}

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}



int main(int argc, char ** argv){
    ros::init(argc, argv, "AEB");
    ros::NodeHandle node;
    ros::Publisher brake_pub = node.advertise<std_msgs::String>("/brake", 1);
    ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
    ros::Subscriber lidar_sub = node.subscribe("/scan", 1, callback);
    ros::Subscriber drive_sub = node.subscribe("/drive", 1, callback_drive);
    ros::Subscriber mode_sub = node.subscribe("/mode", 1, callback_mode);
    ros::Rate loop_rate(50);  

    std_msgs::String msg;
    while(ros::ok()){
        if(correct_pose && m != "f"){           //braking is disabled when in follow-the-gap mode
            msg = stdStringToRosString("m");        //let mux now that maneuver will be performed
            brake_pub.publish(msg);
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            //  Ackermann
            ackermann_msgs::AckermannDrive drive_msg;
            drive_msg.speed = -collision_v;
            drive_msg.steering_angle = alpha;
            //  AckermannStamped
            ackermann_msgs::AckermannDriveStamped drive_st_msg;
            drive_st_msg.header = header;
            drive_st_msg.drive = drive_msg; 
            while(ros::Time::now() - header.stamp < ros::Duration(ttc)){
                command_pub.publish(drive_st_msg); 
            }   
            msg = stdStringToRosString("o");        //let mux know that maneuver is over
            brake_pub.publish(msg); 
        }

        ros::spinOnce();        //pumping subscriber callbacks 
        loop_rate.sleep();
    }


}