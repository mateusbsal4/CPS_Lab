#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "vesc_msg_broker/SetCalibration.h"
#include <robot_localization/SetPose.h>
ros::ServiceClient setPose_srv;

void calibrationDoneCallback(const std_msgs::EmptyConstPtr){
    robot_localization::SetPose srv;
    srv.request.pose.header.stamp = ros::Time::now();
    srv.request.pose.header.frame_id = "odom";
    srv.request.pose.pose.pose.position.x = 0.0;
    srv.request.pose.pose.pose.position.y = 0.0;
    srv.request.pose.pose.pose.position.z = 0.0;
    srv.request.pose.pose.pose.orientation.w = 1.0;
    srv.request.pose.pose.pose.orientation.x = 0.0;
    srv.request.pose.pose.pose.orientation.y = 0.0;
    srv.request.pose.pose.pose.orientation.z = 0.0;
    ROS_INFO("Resetting the pose:");
    if(setPose_srv.call(srv) == true){
        ROS_INFO("done");
    } else {
        ROS_INFO("failed");
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"major_tom") ;
    ros::NodeHandle nh;

    ros::Subscriber calibrationDone_sub = nh.subscribe<std_msgs::Empty>("vesc/calibration_done", 10, &calibrationDoneCallback);
    setPose_srv = nh.serviceClient<robot_localization::SetPose>("/set_pose");
    ros::Publisher startCalibration_pub = nh.advertise<vesc_msg_broker::SetCalibration>("vesc/start_calibration", 10);

    ros::spinOnce();

    ROS_INFO("Waiting for initialization to finish.");
    ros::Duration sleepDuration;
    sleepDuration.sec=2;
    sleepDuration.sleep();

    ROS_INFO("Starting calibration...");
    vesc_msg_broker::SetCalibration setCalibration_msg;
    setCalibration_msg.sampleCount= 1000;
    startCalibration_pub.publish(setCalibration_msg);

    ros::spin();

    return 0;
}
