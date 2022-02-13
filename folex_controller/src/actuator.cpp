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

#include "actuator.hpp"


Actuator::Actuator()
{}

Actuator::~Actuator()
{
  port_handler_->closePort();
}

void Actuator::initActuator()
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(kPortName);
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(kProtocolVer);

  if (port_handler_->openPort() == true)
  {
    ROS_INFO_STREAM("[ACTUATOR] Port opened: " << kPortName);

    port_handler_->setBaudRate(kBaudrate);
    ROS_INFO_STREAM("[ACTUATOR] Set baud rate: " << kBaudrate);

    setDataMap();
  }
  else
  {
    ROS_ERROR_STREAM("[ACTUATOR] Port open failed.");
  }
}

void Actuator::setDataMap()
{
  // Joint number - Dynamixel model
  joints_.insert(std::make_pair(JointNumber::JOINT_1, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(JointNumber::JOINT_2, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_3, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_4, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(JointNumber::JOINT_5, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_6, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_7, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(JointNumber::JOINT_8, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_9, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_10, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(JointNumber::JOINT_11, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(JointNumber::JOINT_12, dxl_ax_.Address::AX_12A));

  // AX address - Data type
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.RETURN_DELAY_TIME, DataType::BYTE));
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.TORQUE_ENABLE, DataType::BYTE));

  // XL address - Data type
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.RETURN_DELAY_TIME, DataType::BYTE));
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.TORQUE_ENABLE, DataType::BYTE));
}

uint16_t Actuator::getDataAddressAX(uint16_t address)
{
  switch (address)
  {
    case DataAddress::RETURN_DELAY_TIME:
      return dxl_ax_.Address::RETURN_DELAY_TIME;
      break;

    case DataAddress::TORQUE_ENABLE:
      return dxl_ax_.Address::TORQUE_ENABLE;
      break;

    default:
      // error
      break;
  }

  return 0;
}

uint16_t Actuator::getDataAddressXL(uint16_t address)
{
  switch (address)
  {
    case DataAddress::RETURN_DELAY_TIME:
      return dxl_xl_.Address::RETURN_DELAY_TIME;
      break;

    case DataAddress::TORQUE_ENABLE:
      return dxl_xl_.Address::TORQUE_ENABLE;
      break;

    default:
      // error
      break;
  }

  return 0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "actuator");

  Actuator actuator;
  actuator.initActuator();

  while (ros::ok())
  {
    ros::spinOnce();
  }
  
  return 0;
}
