#include <ros/ros.h>
#include "vesc_v6_driver/valuesImuReceived.h"
#include "vesc_v6_driver/getImuData.h"
#include "vesc_msg_broker/SetCalibration.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <tf2/utils.h>
#include <math.h>

#define CONST_G_IN_M_P_S2 9.81
#define CONST_DEG_IN_RAD (M_PI / 180.0)
ros::Publisher imu_pub;
ros::Publisher calibrationEnd_pub;

ros::Time lastTs;
double lastAngularVelocityX = 0;
double lastAngularVelocityY = 0;
double lastAngularVelocityZ = 0;
double currentRoll = 0;
double currentPitch = 0;
double currentYaw = 0;

bool calibrationRunning = false;
int calibrationCounter = 0;
double gyroXAgg = 0;
double gyroYAgg = 0;
double gyroZAgg = 0;
double offsetGyroX = 0;
double offsetGyroY = 0;
double offsetGyroZ = 0;
double accelXAgg = 0;
double accelYAgg = 0;
double accelZAgg = 0;
double offsetAccelX = 0;
double offsetAccelY = 0;
double offsetAccelZ = 0;
uint32_t sampleCount = 100;

void imu_callback(const vesc_v6_driver::valuesImuReceivedConstPtr imu_data){
    // IMU has left hand coordinates
    // Getting data: x-axis: forward, y-axis: to left side, z-axis: up
    double angVelX = -imu_data->gyroX * CONST_DEG_IN_RAD;
    double angVelY = imu_data->gyroY * CONST_DEG_IN_RAD;
    double angVelZ = -imu_data->gyroZ * CONST_DEG_IN_RAD;
    double linAccelX = -imu_data->accX * CONST_G_IN_M_P_S2;
    double linAccelY = imu_data->accY * CONST_G_IN_M_P_S2;
    double linAccelZ = -imu_data->accZ * CONST_G_IN_M_P_S2;

    if(calibrationRunning == true){
        gyroXAgg += angVelX;
        gyroYAgg += angVelY;
        gyroZAgg += angVelZ;
        accelXAgg += linAccelX;
        accelYAgg += linAccelY;
        accelZAgg += linAccelZ - 9.81;
        ++calibrationCounter;
        if(calibrationCounter == sampleCount){
            offsetGyroX = gyroXAgg / sampleCount;
            offsetGyroY = gyroYAgg / sampleCount;
            offsetGyroZ = gyroZAgg / sampleCount;
            offsetAccelX = accelXAgg / sampleCount;
            offsetAccelY = accelYAgg / sampleCount;
            offsetAccelZ = accelZAgg / sampleCount;
            lastAngularVelocityX = 0;
            lastAngularVelocityY = 0;
            lastAngularVelocityZ = 0;
            currentRoll = 0;
            currentPitch = 0;
            currentYaw = 0;
            calibrationRunning = false;
            ROS_INFO("Calibration done. Offsets: x=%f, y=%f, z=%f", offsetGyroX, offsetGyroY, offsetGyroZ);
            std_msgs::Empty empty_msg;
            calibrationEnd_pub.publish(empty_msg);
        }
    }

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu";

    msg.linear_acceleration.x = linAccelX - offsetAccelX;
    msg.linear_acceleration.y = linAccelY - offsetAccelY;
    msg.linear_acceleration.z = linAccelZ - offsetAccelZ;

    msg.angular_velocity.x    = angVelX - offsetGyroX;
    msg.angular_velocity.y    = angVelY - offsetGyroY;
    msg.angular_velocity.z    = angVelZ - offsetGyroZ;

    ros::Time now = ros::Time::now();
    double timeDiff_s = (now - lastTs).toSec();
    currentRoll = 0; //+= 0.5 * (msg.angular_velocity.x + lastAngularVelocityX) * timeDiff_s;
    currentPitch = 0; // += 0.5 * (msg.angular_velocity.y + lastAngularVelocityY) * timeDiff_s;
    currentYaw += 0.5 * (msg.angular_velocity.z + lastAngularVelocityZ) * timeDiff_s;
    if(currentRoll > M_PI){
        currentRoll -= 2*M_PI;
    }else if (currentRoll < -M_PI){
        currentRoll += 2*M_PI;
    }
    if(currentPitch > M_PI){
        currentPitch -= 2*M_PI;
    }else if (currentPitch < -M_PI){
        currentPitch += 2*M_PI;
    }
    if(currentYaw > M_PI){
        currentYaw -= 2*M_PI;
    }else if (currentYaw < -M_PI){
        currentYaw += 2*M_PI;
    }
    lastAngularVelocityX = msg.angular_velocity.x;
    lastAngularVelocityY = msg.angular_velocity.y;
    lastAngularVelocityZ = msg.angular_velocity.z;
    lastTs = now;

    tf2::Quaternion imu_quat;
    imu_quat.setRPY(currentRoll, currentPitch, currentYaw);
    imu_quat.normalize();

    msg.orientation_covariance.at(0) = 0.001;
    msg.orientation_covariance.at(4) = 0.001;
    msg.orientation_covariance.at(8) = 0.001;

    msg.angular_velocity_covariance.at(0) = 0.05;
    msg.angular_velocity_covariance.at(4) = 0.05;
    msg.angular_velocity_covariance.at(8) = 0.05;

    msg.linear_acceleration_covariance.at(0) = 1.0;
    msg.linear_acceleration_covariance.at(4) = 1.0;
    msg.linear_acceleration_covariance.at(8) = 0.5;

    tf2::convert(imu_quat, msg.orientation);
    imu_pub.publish(msg);
}

void calibrationCallback(const vesc_msg_broker::SetCalibrationConstPtr msg){
    if(calibrationRunning == true){
        ROS_WARN("Calibration already running.");
        return;
    }
    calibrationRunning = true;
    calibrationCounter = 0;
    gyroXAgg = 0;
    gyroYAgg = 0;
    gyroZAgg = 0;
    accelXAgg = 0;
    accelYAgg = 0;
    accelZAgg = 0;
    sampleCount = msg->sampleCount;
    ROS_INFO("Starting Calibration, please don't touch the vehicle!");
}

int main(int argc, char** argv){

    ros::init(argc, argv, "imu_broker");
    ros::NodeHandle nh;

    lastTs = ros::Time::now();

    ros::Subscriber calibration_sub = nh.subscribe("vesc/start_calibration", 10, &calibrationCallback);

    ros::Subscriber imu_sub = nh.subscribe("vesc/valuesImuReceived", 10, &imu_callback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 10);
    calibrationEnd_pub = nh.advertise<std_msgs::Empty>("vesc/calibration_done", 1);

    ros::Publisher cmd_pub = nh.advertise<vesc_v6_driver::getImuData>("vesc/getImuData", 10);
    vesc_v6_driver::getImuData cmd_msg;
    cmd_msg.mask = 0xFFFFFFFF;

    ros::Rate rate(100);
    while(ros::ok()){
        cmd_pub.publish(cmd_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
