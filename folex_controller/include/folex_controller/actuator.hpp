/******************************************************************************
* Copyright 2022 Hyunwook Choi
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************/

#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <iostream>
#include <ros/ros.h>


class Actuator
{
private:
  // ROS
  ros::NodeHandle nh_;

  // Dynamixel
  const char *kPortName = "/dev/U2D2";
  const float kProtocolVer = 1.0F;
  const uint32_t kBaudrate = 1000000;
  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;

public:
  Actuator();
  ~Actuator();

  void initActuator();
};

# endif  // ACTUATOR_HPP
