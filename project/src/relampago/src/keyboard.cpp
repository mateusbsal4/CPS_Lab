#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//                         0:            1:          2:        3:
//float mapping[4][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 0.0}};
//remember in c array[i] is equivalent to *(array+i)

//extension to include braking
//                          0:            1:          2:        3:             4:
float mapping[5][2] =  {{0.5, 0.0}, {0.5, -0.4}, {0.5, 0.4}, {-0.5, 0.0}, {0.0, 0.0}};

float angle_limit = 0.3;

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

unsigned keyToIndex(char message) {
  unsigned res;
  if (message == 'w') {
    res = 0;
  }
  else if (message == 'd') {
    res = 1;
  }
  else if (message == 'a') {
    res = 2;
  }
  else if (message == 's') {
    res = 3;
  }
  else if(message == ' '){   //include braking
    res = 4;
  }
  else {
    res = 4;
  }
  return res;
}

int main(int argc, char *argv[]) {  
  ros::init(argc, argv, "keyboard");

  ros::NodeHandle n;

  //ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_keyboard", 1);
  ros::Publisher mode_pub = n.advertise<std_msgs::String>("/mode", 1);
  float speed = 0.0;
  float angle = 0.0;
  while(ros::ok()) {
    char input  = getch();
    std::string desired_mode = (input=='p')?"p":((input=='g')?"g":"k");      //setting the mode to follow the gap or keyboard operation
    //printf("Mode %s \n", desired_mode.c_str());
    unsigned index = keyToIndex(input);
    speed = mapping[index][0];
    angle = mapping[index][1];

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed;
    drive_msg.steering_angle = angle*angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);

    //publish mode msg 
    std_msgs::String mode_msg = stdStringToRosString(desired_mode);
    mode_pub.publish(mode_msg);
    command_pub.publish(drive_st_msg);
  }
  return 0;
}
