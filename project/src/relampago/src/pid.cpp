#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
int length_ranges = 0;
float min_range = 0;
std::vector<float> full_ranges;
int min_index = 0;
float input_yaw = 0;
float d_objective = 0;
float speed = 0.5;
float e = 0;
float e_old = 0;
float K_d = 1.0;
float K_i = 0.00;
float sum_err = 0;
float L= 0.5/50;

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
    full_ranges = msg->ranges;
    std::vector<float> ranges(full_ranges.begin() + 80, full_ranges.end() - 80);        //cropping to compensate for lidar noise
    length_ranges = ranges.size();
    std::vector<float> ranges_right(full_ranges.begin()+ 80, full_ranges.begin()+540);
    std::vector<float> ranges_left(full_ranges.begin()+540, full_ranges.end()-80);    
    d_objective = (min_element(ranges_right)+min_element(ranges_left))/2;       //target distance from the wall - allows to compensate for varying track widths
    min_range = min_element(ranges);                //minimum distance measured by the lidar
    min_index = min_element_idx(ranges);
    e = min_range - d_objective - L*sin(input_yaw);
    float K_p = (min_index > length_ranges/2)?2.5f:-2.5f;
    sum_err = ( min_index > length_ranges/2)? sum_err + e: sum_err -e;
   // printf("Objective distance: %f \n", d_objective);
   // printf("Min range: %f \n", min_range);
    input_yaw = K_p*e + K_d*(e_old-e) + K_i * sum_err;
   // printf("Input yaw: %f\n", input_yaw);
    e_old = e;
}






int main(int argc, char ** argv){
  ros::init(argc, argv, "pid");
  ros::NodeHandle n;
  //ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop", 1);       //when running standalone
  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_pid", 1);         //when running with the mux
  ros::Subscriber lidar_sub = n.subscribe("/scan", 1, callback);
  ros::Rate loop_rate(50);

   while(ros::ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        //  Ackermann
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.speed = speed;
        drive_msg.steering_angle = input_yaw;
        //  AckermannStamped
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        drive_st_msg.header = header;
        drive_st_msg.drive = drive_msg;
        command_pub.publish(drive_st_msg);
        ros::spinOnce();
    }
}
