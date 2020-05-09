//

#include "folex_dynamixel_driver.h"


FolexDynamixelDriver::FolexDynamixelDriver()
: baudrate_(BAUDRATE), protocol_version_(PROTOCOL_VERSION)
{
  // Dynamixel Model Number and ID
  dxl_array_.insert(std::make_pair(JOINT_1, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_2, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_3, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_4, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_5, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_6, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_7, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_8, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_9, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_10, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_11, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_12, AX_12A));

  // Dynamixel Address and Size
  dxl_xl_address_array_.insert(std::make_pair(ADDR_XL_TORQUE_ENABLE, BYTE));
  dxl_xl_address_array_.insert(std::make_pair(ADDR_XL_GOAL_POSITION, DWORD));
  dxl_xl_address_array_.insert(std::make_pair(ADDR_XL_GOAL_VELOCITY, DWORD));
  dxl_ax_address_array_.insert(std::make_pair(ADDR_AX_TORQUE_ENABLE, BYTE));
  dxl_ax_address_array_.insert(std::make_pair(ADDR_AX_GOAL_POSITION, WORD));
  dxl_ax_address_array_.insert(std::make_pair(ADDR_AX_MOVING_SPEED, WORD));
}

FolexDynamixelDriver::~FolexDynamixelDriver()
{
  close();
}

bool FolexDynamixelDriver::init()
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    return false;
  }

  // Set port baud rate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    return false;
  }

  return true;
}

void FolexDynamixelDriver::close()
{
  // Close port
  portHandler_->closePort();
}

void FolexDynamixelDriver::addDynamixel(std::string name, std::string parent_name, std::string child_name)
{


}

bool FolexDynamixelDriver::setTorque(uint8_t id, bool onoff)
{
  bool result = false;
  uint8_t dxl_error = 0;

  if (dxl_array_.find(id)->second == XL430_W250)
  {
    result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL_TORQUE_ENABLE, onoff, &dxl_error);
  }
  else if (dxl_array_.find(id)->second == AX_12A)
  {
    result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_AX_TORQUE_ENABLE, onoff, &dxl_error);
  }
  else
  {
    // error
  }
  
  return result;
}

bool FolexDynamixelDriver::enableDynamixel()
{
  bool result = false;
  std::map<uint8_t, uint16_t>::iterator it_dynamixel_array;

  for (it_dynamixel_array = dxl_array_.begin(); it_dynamixel_array != dxl_array_.end(); it_dynamixel_array++)
  {
    result = setTorque(it_dynamixel_array->first, TORQUE_ENABLE);
  }

  return result;
}

bool FolexDynamixelDriver::disableDynamixel()
{
  bool result = false;
  std::map<uint8_t, uint16_t>::iterator it_dynamixel_array;

  for (it_dynamixel_array = dxl_array_.begin(); it_dynamixel_array != dxl_array_.end(); it_dynamixel_array++)
  {
    result = setTorque(it_dynamixel_array->first, TORQUE_DISABLE);
  }

  return result;
}

void FolexDynamixelDriver::moveDynamixel(double j1, double j2, double j3, double time)
{
  
  
}

bool FolexDynamixelDriver::writeValue(uint8_t id, uint16_t address, uint32_t value)
{
  bool result = false;
  uint8_t dxl_error = 0;

  // XL430 or AX-12A
  if (dxl_array_.find(id)->second == XL430_W250)
  {
    switch (dxl_xl_address_array_.find(address)->second)
    {
    case BYTE:
      result = packetHandler_->write1ByteTxRx(portHandler_, id, address, value, &dxl_error);
      break;

    case WORD:
      result = packetHandler_->write2ByteTxRx(portHandler_, id, address, value, &dxl_error);
      break;

    case DWORD:
      result = packetHandler_->write4ByteTxRx(portHandler_, id, address, value, &dxl_error);
      break;
    
    default:
      result = false;
      break;
    }
  }
  else if (dxl_array_.find(id)->second == AX_12A)
  {
    switch (dxl_ax_address_array_.find(address)->second)
    {
    case BYTE:
      result = packetHandler_->write1ByteTxRx(portHandler_, id, address, value, &dxl_error);
      break;

    case WORD:
      result = packetHandler_->write2ByteTxRx(portHandler_, id, address, value, &dxl_error);
      break;
    
    default:
      result = false;
      break;
    }
  }
  else
  {
    // error
  }

  return result;
}

