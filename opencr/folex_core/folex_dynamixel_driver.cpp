// 

#include "folex_dynamixel_driver.h"


FolexDynamixelDriver::FolexDynamixelDriver()
: baudrate_(BAUDRATE), protocol_version_(PROTOCOL_VERSION),
  joint_1_id_(JOINT_1),
  joint_2_id_(JOINT_2),
  joint_3_id_(JOINT_3)
{
  torque_ = false;
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

  groupSyncWriteXL_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_XL_GOAL_POSITION, LEN_XL_GOAL_POSITION);
  groupSyncWritePositionAX_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
  groupSyncWriteRpmAX_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_AX_MOVING_SPEED, LEN_AX_MOVING_SPEED);

  return true;
}

void FolexDynamixelDriver::close()
{
  // Close port
  portHandler_->closePort();
}

bool FolexDynamixelDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, JOINT_1, ADDR_XL_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }

  return true;
}

bool FolexDynamixelDriver::writePosition()
{
  bool dxl_xl_addparam_result;
  bool dxl_ax_addparam_result;

  uint8_t xl_data_byte[4];
  uint8_t ax_data_byte[2];
  uint8_t ax_rpm_data_byte[2];

  xl_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(XL_TEST_POSITION));
  xl_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(XL_TEST_POSITION));
  xl_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(XL_TEST_POSITION));
  xl_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(XL_TEST_POSITION));

  ax_data_byte[0] = DXL_LOBYTE(AX_TEST_POSITION);
  ax_data_byte[1] = DXL_HIBYTE(AX_TEST_POSITION);

  ax_rpm_data_byte[0] = DXL_LOBYTE(AX_TEST_RPM);
  ax_rpm_data_byte[1] = DXL_HIBYTE(AX_TEST_RPM);

  setTorque(TORQUE_ENABLE);

  dxl_xl_addparam_result = groupSyncWriteXL_->addParam(joint_1_id_, xl_data_byte);
  if (dxl_xl_addparam_result != true)
  {
    return false;
  }

  dxl_ax_addparam_result = groupSyncWritePositionAX_->addParam(joint_2_id_, ax_data_byte);
  if (dxl_ax_addparam_result != true)
  {
    return false;
  }

  dxl_ax_addparam_result = groupSyncWriteRpmAX_->addParam(joint_2_id_, ax_rpm_data_byte);
  if (dxl_ax_addparam_result != true)
  {
    return false;
  }

  dxl_xl_addparam_result = groupSyncWriteXL_->txPacket();
  if (dxl_xl_addparam_result != COMM_SUCCESS)
  {
    return false;
  }

  dxl_ax_addparam_result = groupSyncWritePositionAX_->txPacket();
  if (dxl_ax_addparam_result != COMM_SUCCESS)
  {
    return false;
  }

  dxl_ax_addparam_result = groupSyncWriteRpmAX_->txPacket();
  if (dxl_ax_addparam_result != COMM_SUCCESS)
  {
    return false;
  }

  groupSyncWriteXL_->clearParam();
  return true;

  groupSyncWritePositionAX_->clearParam();
  return true;

  groupSyncWriteRpmAX_->clearParam();
  return true;
}
