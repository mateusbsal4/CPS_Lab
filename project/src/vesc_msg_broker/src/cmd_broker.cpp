#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vesc_v6_driver/setRpm.h>
#include <vesc_v6_driver/setServoPos.h>
ros::Publisher rpm_pub;
ros::Publisher servo_pub;

void speedcallback(const std_msgs::Float64ConstPtr &msg){
    //if(msg->data > -1 && msg->data < 1)
    //{
    //    return;
    //}
    vesc_v6_driver::setRpm rpm_msg;
    rpm_msg.rpm = static_cast<int32_t>(msg->data);
    rpm_pub.publish(rpm_msg);
}

void servocallback(const std_msgs::Float64ConstPtr &msg){
    vesc_v6_driver::setServoPos servo_msg;
    servo_msg.pos = msg->data;
    servo_pub.publish(servo_msg);
}
int main(int argc,char **argv){
    ros::init(argc,argv,"cmd_broker");
    ros::NodeHandle nh;
    rpm_pub = nh.advertise<vesc_v6_driver::setRpm>("vesc/setRpm",10);
    servo_pub = nh.advertise<vesc_v6_driver::setServoPos>("vesc/setServoPos", 10);
    ros::Subscriber rpm_sub = nh.subscribe("commands/motor/speed",10,speedcallback);
    ros::Subscriber servo_sub = nh.subscribe("commands/servo/position",10,servocallback);
    ros::spin();
    return 0;
}

