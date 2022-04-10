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
    std::cout << "[ACTUATOR] Port opened: " << kPortName << std::endl;

    port_handler_->setBaudRate(kBaudrate);
    std::cout << "[ACTUATOR] Set baud rate: " << kBaudrate << std::endl;

    setDataMap();
    setReturnDelayTime(0);
  }
  else
  {
    std::cout << "[ACTUATOR] Port open failed." << std::endl;
  }
}

void Actuator::enableActuator(uint8_t id)
{
  if (id == Joint::JOINT_ALL)
  {
    writeDataALL(DataAddress::TORQUE_ENABLE, DataPreset::TORQUE_ON);
  }
  else if (id < Joint::JOINT_ALL)
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
  if (id == Joint::JOINT_ALL)
  {
    writeDataALL(DataAddress::TORQUE_ENABLE, DataPreset::TORQUE_OFF);
  }
  else if (id < Joint::JOINT_ALL)
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
  joints_.insert(std::make_pair(Joint::JOINT_1, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(Joint::JOINT_2, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_3, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_4, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(Joint::JOINT_5, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_6, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_7, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(Joint::JOINT_8, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_9, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_10, dxl_xl_.Address::XL430_W250));
  joints_.insert(std::make_pair(Joint::JOINT_11, dxl_ax_.Address::AX_12A));
  joints_.insert(std::make_pair(Joint::JOINT_12, dxl_ax_.Address::AX_12A));

  // AX address - Data type
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.RETURN_DELAY_TIME, DataType::BYTE));
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.TORQUE_ENABLE, DataType::BYTE));
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.MOVING_SPEED, DataType::WORD));
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.PRESENT_POSITION, DataType::WORD));
  dxl_ax_.address_map_.insert(std::make_pair(dxl_ax_.PRESENT_SPEED, DataType::WORD));

  // XL address - Data type
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.RETURN_DELAY_TIME, DataType::BYTE));
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.TORQUE_ENABLE, DataType::BYTE));
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.GOAL_VELOCITY, DataType::DWORD));
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.PRESENT_VELOCITY, DataType::DWORD));
  dxl_xl_.address_map_.insert(std::make_pair(dxl_xl_.PRESENT_POSITION, DataType::DWORD));
}

void Actuator::setReturnDelayTime(uint8_t delay)
{
  writeData(Joint::JOINT_ALL, DataAddress::RETURN_DELAY_TIME, delay);
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

    case DataAddress::PRESENT_ANGLE:
      return dxl_ax_.Address::PRESENT_POSITION;
      break;

    case DataAddress::PRESENT_VELOCITY:
      return dxl_ax_.Address::PRESENT_SPEED;
      break;

    case DataAddress::TARGET_VELOCITY:
      return dxl_ax_.Address::MOVING_SPEED;
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

    case DataAddress::PRESENT_ANGLE:
      return dxl_xl_.Address::PRESENT_POSITION;
      break;

    case DataAddress::PRESENT_VELOCITY:
      return dxl_xl_.Address::PRESENT_VELOCITY;
      break;

    case DataAddress::TARGET_VELOCITY:
      return dxl_xl_.Address::GOAL_VELOCITY;
      break;

    default:
      // error
      break;
  }

  return 0;
}

void Actuator::readDataAX(uint8_t id, uint16_t address, uint32_t *data)
{
  if (dxl_ax_.address_map_.find(address)->second == DataType::BYTE)
  {
    packet_handler_->read1ByteTxRx(port_handler_, id, address, &dxl_ax_.buffer_uint8_, &dxl_ax_.error_);
    data[id] = dxl_ax_.buffer_uint8_;
  }
  else if (dxl_ax_.address_map_.find(address)->second == DataType::WORD)
  {
    packet_handler_->read2ByteTxRx(port_handler_, id, address, &dxl_ax_.buffer_uint16_, &dxl_ax_.error_);
    data[id] = dxl_ax_.buffer_uint16_;
  }
  else
  {
    // error
  }
}

void Actuator::readDataXL(uint8_t id, uint16_t address, uint32_t *data)
{
  if (dxl_xl_.address_map_.find(address)->second == DataType::BYTE)
  {
    packet_handler_->read1ByteTxRx(port_handler_, id, address, &dxl_xl_.buffer_uint8_, &dxl_xl_.error_);
    data[id] = dxl_xl_.buffer_uint8_;
  }
  else if (dxl_xl_.address_map_.find(address)->second == DataType::WORD)
  {
    packet_handler_->read2ByteTxRx(port_handler_, id, address, &dxl_xl_.buffer_uint16_, &dxl_xl_.error_);
    data[id] = dxl_xl_.buffer_uint16_;
  }
  else if (dxl_xl_.address_map_.find(address)->second == DataType::DWORD)
  {
    packet_handler_->read4ByteTxRx(port_handler_, id, address, &dxl_xl_.buffer_uint32_, &dxl_xl_.error_);
    data[id] = dxl_xl_.buffer_uint32_;
  }
  else
  {
    // error
  }
}

void Actuator::readPresentAngle()
{
  for (auto p : joints_)
  {
    if (p.second == dxl_ax_.Address::AX_12A)
    {
      readDataAX(p.first, getDataAddressAX(PRESENT_ANGLE), Joint::present_angle_value);
    }
    else if (p.second == dxl_xl_.Address::XL430_W250)
    {
      readDataXL(p.first, getDataAddressXL(PRESENT_ANGLE), Joint::present_angle_value);
    }
    else
    {
      // error
    }
  }
}

void Actuator::readPresentVelocity()
{
  for (auto p : joints_)
  {
    if (p.second == dxl_ax_.Address::AX_12A)
    {
      readDataAX(p.first, getDataAddressAX(PRESENT_VELOCITY), Joint::present_velocity_value);
    }
    else if (p.second == dxl_xl_.Address::XL430_W250)
    {
      readDataXL(p.first, getDataAddressXL(PRESENT_VELOCITY), Joint::present_velocity_value);
    }
    else
    {
      // error
    }
  }
}

void Actuator::writeData(uint8_t id, uint16_t address, uint32_t data)
{
  if (id == Joint::JOINT_ALL)
  {
    writeDataALL(address, data);
  }
  else if (id < Joint::JOINT_ALL)
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

void Actuator::writeTargetVelocity()
{
  for (auto p : joints_)
  {
    if (p.second == dxl_ax_.Address::AX_12A)
    {
      writeDataAX(p.first, getDataAddressAX(DataAddress::TARGET_VELOCITY), Joint::target_velocity_value[p.first]);
    }
    else if (p.second == dxl_xl_.Address::XL430_W250)
    {
      writeDataXL(p.first, getDataAddressXL(DataAddress::TARGET_VELOCITY), Joint::target_velocity_value[p.first]);
    }
    else
    {
      // error
    }
  }
}

void Actuator::convertRadianToValue(float (&radian)[12], uint32_t (&value)[12])
{
  for (auto p : joints_)
  {
    if (p.second == dxl_ax_.Address::AX_12A)
    {
      if (p.first == Joint::JOINT_5 || p.first == Joint::JOINT_6 || p.first == Joint::JOINT_11 || p.first == Joint::JOINT_12)
      {
        value[p.first] = ((-1.0F * radian[p.first]) + dxl_ax_.kHomeAngleRadian) / dxl_ax_.kRadianPerValue;
      }
      else
      {
        value[p.first] = (radian[p.first] + dxl_ax_.kHomeAngleRadian) / dxl_ax_.kRadianPerValue;
      }
    }
    else if (p.second == dxl_xl_.Address::XL430_W250)
    {
      if (p.first == Joint::JOINT_7 || p.first == Joint::JOINT_10)
      {
        value[p.first] = ((-1.0F * radian[p.first]) + dxl_xl_.kHomeAngleRadian) / dxl_xl_.kRadianPerValue;
      }
      else
      {
        value[p.first] = (radian[p.first] + dxl_xl_.kHomeAngleRadian) / dxl_xl_.kRadianPerValue;
      }
    }
    else
    {
      // error
    }
  }
}

void Actuator::convertValueToRadian(uint32_t (&value)[12], float (&radian)[12])
{
  for (auto p : joints_)
  {
    if (p.second == dxl_ax_.Address::AX_12A)
    {
      if (p.first == Joint::JOINT_5 || p.first == Joint::JOINT_6 || p.first == Joint::JOINT_11 || p.first == Joint::JOINT_12)
      {
        radian[p.first] = -1.0F * ((value[p.first] - dxl_ax_.kHomeAngleValue) / dxl_ax_.kValuePerRadian);
      }
      else
      {
        radian[p.first] = (value[p.first] - dxl_ax_.kHomeAngleValue) / dxl_ax_.kValuePerRadian;
      }
    }
    else if (p.second == dxl_xl_.Address::XL430_W250)
    {
      if (p.first == Joint::JOINT_7 || p.first == Joint::JOINT_10)
      {
        radian[p.first] = -1.0F * ((value[p.first] - dxl_xl_.kHomeAngleValue) / dxl_xl_.kValuePerRadian);
      }
      else
      {
        radian[p.first] = (value[p.first] - dxl_xl_.kHomeAngleValue) / dxl_xl_.kValuePerRadian;
      }
    }
    else
    {
      // error
    }
  }
}
