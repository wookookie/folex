//

#ifndef FOLEX_CORE_H
#define FOLEX_CORE_H

#include <ros.h>
#include <std_msgs/String.h>

#include "folex.h"


// ROS NodeHandle
ros::NodeHandle nh;

std_msgs::String dxl_status;

// Class
Folex folex;
FolexDiagnosis diagnosis;  // OpenCR diagnosis
FolexDynamixelDriver dynamixel_driver;  // Dynamixel driver

// Publisher
ros::Publisher dxl_status_pub("dynamixel_status", &dxl_status);


// Functions
void findDynamixel();
void sendLogMsg();
void startLogMsg();


#endif // FOLEX_CORE_H
