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
    setReturnDelayTime(0);
  }
  else
  {
    ROS_ERROR_STREAM("[ACTUATOR] Port open failed.");
  }
}

void Actuator::enableActuator(uint8_t id)
{
  if (id == JointNumber::JOINT_ALL)
  {
    writeDataALL(DataAddress::TORQUE_ENABLE, DataPreset::TORQUE_ON);
  }
  else if (id > 0)
  {
    if (joints_.find(id)->second == dxl_ax_.Address::AX_12A)
    {
      writeDataAX(id, getDataAddressAX(DataAddress::TORQUE_ENABLE), DataPreset::TORQUE_ON);
    }
    else if (joints_.find(id)->second == dxl_xl_.Address::XL430_W250)
    {
      writeDataXL(id, getDataAddressXL(DataAddress::TORQUE_ENABLE), DataPreset::TORQUE_ON);
    }
  }
  else
  {
    // error
  }
}

void Actuator::disableActuator(uint8_t id)
{
  if (id == JointNumber::JOINT_ALL)
  {
    writeDataALL(DataAddress::TORQUE_ENABLE, DataPreset::TORQUE_OFF);
  }
  else if (id > 0)
  {
    if (joints_.find(id)->second == dxl_ax_.Address::AX_12A)
    {
      writeDataAX(id, getDataAddressAX(DataAddress::TORQUE_ENABLE), DataPreset::TORQUE_OFF);
    }
    else if (joints_.find(id)->second == dxl_xl_.Address::XL430_W250)
    {
      writeDataXL(id, getDataAddressXL(DataAddress::TORQUE_ENABLE), DataPreset::TORQUE_OFF);
    }
  }
  else
  {
    // error
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

void Actuator::setReturnDelayTime(uint8_t delay)
{
  writeData(JointNumber::JOINT_ALL, DataAddress::RETURN_DELAY_TIME, delay);
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

void Actuator::writeData(uint8_t id, uint16_t address, uint32_t data)
{
  if (id == JointNumber::JOINT_ALL)
  {
    writeDataALL(address, data);
  }
  else if (id > 0)
  {
    if (joints_.find(id)->second == dxl_ax_.Address::AX_12A)
    {
      writeDataAX(id, getDataAddressAX(address), data);
    }
    else if (joints_.find(id)->second == dxl_xl_.Address::XL430_W250)
    {
      writeDataXL(id, getDataAddressXL(address), data);
    }
  }
  else
  {
    // error
  }
}

void Actuator::writeDataALL(uint16_t address, uint32_t data)
{
  for (auto p : joints_)
  {
    if (p.second == dxl_ax_.Address::AX_12A)
    {
      writeDataAX(p.first, getDataAddressAX(address), data);
    }
    else if (p.second == dxl_xl_.Address::XL430_W250)
    {
      writeDataXL(p.first, getDataAddressXL(address), data);
    }
    else
    {
      // error
    }
  }
}

void Actuator::writeDataAX(uint8_t id, uint16_t address, uint32_t data)
{
  if (dxl_ax_.address_map_.find(address)->second == DataType::BYTE)
  {
    packet_handler_->write1ByteTxRx(port_handler_, id, address, data, &dxl_ax_.error_);
  }
  else if (dxl_ax_.address_map_.find(address)->second == DataType::WORD)
  {
    packet_handler_->write2ByteTxRx(port_handler_, id, address, data, &dxl_ax_.error_);
  }
  else
  {
    // error
  }
}

void Actuator::writeDataXL(uint8_t id, uint16_t address, uint32_t data)
{
  if (dxl_xl_.address_map_.find(address)->second == DataType::BYTE)
  {
    packet_handler_->write1ByteTxRx(port_handler_, id, address, data, &dxl_xl_.error_);
  }
  else if (dxl_xl_.address_map_.find(address)->second == DataType::WORD)
  {
    packet_handler_->write2ByteTxRx(port_handler_, id, address, data, &dxl_xl_.error_);
  }
  else if (dxl_xl_.address_map_.find(address)->second == DataType::DWORD)
  {
    packet_handler_->write4ByteTxRx(port_handler_, id, address, data, &dxl_xl_.error_);
  }
  else
  {
    // error
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "actuator");

  Actuator actuator;
  actuator.initActuator();
  actuator.enableActuator(JointNumber::JOINT_ALL);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  
  return 0;
}
