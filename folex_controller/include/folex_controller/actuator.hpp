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
#include <map>
#include <ros/ros.h>


enum JointNumber
{
  JOINT_ALL,
  JOINT_1, JOINT_2, JOINT_3,
  JOINT_4, JOINT_5, JOINT_6,
  JOINT_7, JOINT_8, JOINT_9,
  JOINT_10, JOINT_11, JOINT_12
};

enum DataType
{
  BYTE = 1,
  WORD = 2,
  DWORD = 4
};

enum DataAddress
{
  RETURN_DELAY_TIME,
  TORQUE_ENABLE
};

class DynamixelAX
{
public:
  enum Address
  {
    AX_12A = 12,
    RETURN_DELAY_TIME = 5,
    TORQUE_ENABLE = 24
  };
  std::map<uint8_t, uint8_t> address_map_;
  uint8_t error_;
};

class DynamixelXL
{
public:
  enum Address
  {
    XL430_W250 = 1060,
    RETURN_DELAY_TIME = 9,
    TORQUE_ENABLE = 64
  };
  std::map<uint8_t, uint8_t> address_map_;
  uint8_t error_;
};

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
  DynamixelAX dxl_ax_;
  DynamixelXL dxl_xl_;

  // Joint
  std::map<uint8_t, uint16_t> joints_;

public:
  Actuator();
  ~Actuator();

  void initActuator();

  void setDataMap();
  uint16_t getDataAddressAX(uint16_t address);
  uint16_t getDataAddressXL(uint16_t address);
};

# endif  // ACTUATOR_HPP
