// 

#include "folex_dynamixel_driver.h"


FolexDynamixelDriver::FolexDynamixelDriver()
: baudrate_(BAUDRATE), protocol_version_(PROTOCOL_VERSION), left_front_id_(LEFT_FRONT)
{

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

  groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);

  return true;
}

void FolexDynamixelDriver::close()
{
  // Close port
  portHandler_->closePort();
}

bool FolexDynamixelDriver::writePosition()
{
  bool dxl_addparam_result;
  uint8_t left_ax_data_byte[2];

  left_ax_data_byte[0] = DXL_LOBYTE(AX_TEST_POSITION);
  left_ax_data_byte[1] = DXL_HIBYTE(AX_TEST_POSITION);

  dxl_addparam_result = groupSyncWrite_->addParam(left_front_id_, left_ax_data_byte);
  if (dxl_addparam_result != true)
  {
    return false;
  }

  dxl_addparam_result = groupSyncWrite_->txPacket();
  if (dxl_addparam_result != COMM_SUCCESS)
  {
    return false;
  }

  groupSyncWrite_->clearParam();
  return true;  
}
