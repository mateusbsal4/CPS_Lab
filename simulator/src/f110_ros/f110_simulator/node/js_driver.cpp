#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>


#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <string.h>
#include <stdio.h>
#include <map>

#define JSKEY_A     0x001
#define JSKEY_B     0x101
#define JSKEY_X     0x201
#define JSKEY_Y     0x301
#define JSKEY_LB    0x401
#define JSKEY_RB    0x501
#define JSKEY_MODE  0x601
#define JSKEY_START 0x701
#define JSKEY_HOME  0x801

#define JSKEY_LEFTSTICK  0x901
#define JSKEY_RIGHTSTICK 0xa01

#define JSKEY_PRESS 0x001
#define JSKEY_RELEASE 0x0

#define JSKEY_LT 0x202
#define JSKEY_RT 0x502

#define JSKEY_CROSS_HORIZONT 0x602
#define JSKEY_CROSS_VERTICAL 0x702
#define JSKEY_LEFTSTICK_HORIZONT 0x002
#define JSKEY_LEFTSTICK_VERTICAL 0x102
#define JSKEY_RIGHTSTICK_HORIZONT 0x302
#define JSKEY_RIGHTSTICK_VERTICAL 0x402

#define JSKEY_CROSS_LOW_VALUE 0xffff8001
#define JSKEY_CROSS_HIGH_VALUE 0x7fff
#define JSKEY_CROSS_RELEASED 0x0

ssize_t n;
int fd;
int buf[2];

int main(int argc, char** argv) {
    //ROS initilization
    ros::init(argc, argv, "js_driver");
    ros::NodeHandle node;

    ros::Publisher command_pub = node.advertise<geometry_msgs::Vector3>("js", 1);
    
    char dev[] = "/dev/input/js0";
    memset(buf, 0, sizeof buf);

    // Initialization
    fd = open("/dev/input/js0", O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Cannot open %s: %s.\n", dev, strerror(errno));
        return EXIT_FAILURE;
    }
    geometry_msgs::Vector3 msg;
    msg.z = 0.0;

    // Main loop
    while (ros::ok()) {
        memset(buf, 0, sizeof buf);
        n = read(fd, &buf, sizeof buf);
        n = n / sizeof(int);
        if (n == (ssize_t)-1) {
            if (errno == EINTR)
                continue;
            else
                break;
        }

        unsigned short btn = buf[1] >> 16;
        short val = (short)(buf[1] & 0xffff);
	float res = 0.0;

        if (btn == JSKEY_LT || btn == JSKEY_RT) {
            unsigned short prs_val = val + 32768;
            res = (unsigned short) (((long)prs_val)*100/65536);
	}
	else if (btn == JSKEY_LEFTSTICK_HORIZONT || btn == JSKEY_LEFTSTICK_VERTICAL ||
                 btn == JSKEY_RIGHTSTICK_HORIZONT || btn == JSKEY_RIGHTSTICK_VERTICAL)
            res = val*100/32767;
	
	msg.x = btn;
	msg.y = res;

	command_pub.publish(msg);
    }

    fflush(stdout);
    fprintf(stderr, "%s.\n", strerror(errno));
    return 0;
}
