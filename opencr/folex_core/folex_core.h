//

#ifndef FOLEX_CORE_H
#define FOLEX_CORE_H

#include <ros.h>
#include <std_msgs/String.h>

#include "folex.h"


// ROS NodeHandle
ros::NodeHandle nh;

std_msgs::String msg;
std_msgs::String dxl_status;

// Class
Folex folex;

// Publisher
ros::Publisher chatter_pub("chatter", &msg);

ros::Publisher dxl_status_pub("dynamixel_status", &dxl_status);

static uint32_t pre_time;


// Functions
void findDynamixel();
void sendLogMsg();
void startLogMsg();


#endif // FOLEX_CORE_H
