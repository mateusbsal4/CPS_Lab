#include <ros/ros.h>

#include <vesc_msgs/VescStateStamped.h>
#include <vesc_v6_driver/getValues.h>
#include <vesc_v6_driver/valuesReceived.h>

ros::Publisher state_pub;

void state_callback(const vesc_v6_driver::valuesReceivedConstPtr &msg){
    vesc_msgs::VescStateStamped state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.header.frame_id = "base_link";

    state_msg.state.voltage_input = msg->v_in;
    state_msg.state.temperature_pcb = msg->temp_mos;
    state_msg.state.current_motor = msg->current_motor;
    state_msg.state.current_input = msg->current_in;
    state_msg.state.speed = msg->rpm;
    state_msg.state.duty_cycle = msg->duty_now;
    state_msg.state.charge_drawn = msg->amp_hours;
    state_msg.state.charge_regen = msg->amp_hours_charged;
    state_msg.state.energy_drawn = msg->watt_hours;
    state_msg.state.energy_regen = msg->watt_hours_charged;
    state_msg.state.displacement = msg->position;
    state_msg.state.distance_traveled = msg->tachometer;
    state_msg.state.fault_code = msg->fault_code;

    state_pub.publish(state_msg);
}

int main(int argc,char **argv){

    ros::init(argc,argv,"state_broker");
    ros::NodeHandle nh;

    state_pub = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);
    ros::Subscriber values_sub = nh.subscribe<vesc_v6_driver::valuesReceived>("vesc/valuesReceived", 10, state_callback);
    ros::Publisher getValues_pub = nh.advertise<vesc_v6_driver::getValues>("vesc/getValues", 10);

    ros::Rate rate(30);
    while(ros::ok()){
        vesc_v6_driver::getValues getValues_msg;
        getValues_pub.publish(getValues_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

