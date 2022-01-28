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
  }
  else
  {
    ROS_ERROR_STREAM("[ACTUATOR] Port open failed.");
  }
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
