//

#include "folex_dynamixel_driver.h"

// Temp
#include "kinematics.h"


FolexDynamixelDriver::FolexDynamixelDriver()
: baudrate_(BAUDRATE), protocol_version_(PROTOCOL_VERSION)
{
  // Dynamixel Model Number and ID
  dxl_array_.insert(std::make_pair(JOINT_1, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_2, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_3, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_4, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_5, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_6, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_7, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_8, AX_12A));
  dxl_array_.insert(std::make_pair(JOINT_9, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_10, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_11, XL430_W250));
  dxl_array_.insert(std::make_pair(JOINT_12, XL430_W250));

  // Dynamixel Address and Size
  dxl_ax_address_array_.insert(std::make_pair(ADDR_AX_TORQUE_ENABLE, BYTE));
  dxl_ax_address_array_.insert(std::make_pair(ADDR_AX_GOAL_POSITION, WORD));
  dxl_ax_address_array_.insert(std::make_pair(ADDR_AX_MOVING_SPEED, WORD));
  dxl_xl_address_array_.insert(std::make_pair(ADDR_XL_TORQUE_ENABLE, BYTE));
  dxl_xl_address_array_.insert(std::make_pair(ADDR_XL_GOAL_POSITION, DWORD));
  dxl_xl_address_array_.insert(std::make_pair(ADDR_XL_PROFILE_VELOCITY, DWORD));
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

  // TODO : Adverties status
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

  if (dxl_array_.find(id)->second == AX_12A)
  {
    result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_AX_TORQUE_ENABLE, onoff, &dxl_error);
  }
  else if (dxl_array_.find(id)->second == XL430_W250)
  {
    result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XL_TORQUE_ENABLE, onoff, &dxl_error);
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
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;

  for (it_dxl_array = dxl_array_.begin(); it_dxl_array != dxl_array_.end(); it_dxl_array++)
  {
    result = setTorque(it_dxl_array->first, TORQUE_ENABLE);
  }

  return result;
}

bool FolexDynamixelDriver::disableDynamixel()
{
  bool result = false;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;

  for (it_dxl_array = dxl_array_.begin(); it_dxl_array != dxl_array_.end(); it_dxl_array++)
  {
    result = setTorque(it_dxl_array->first, TORQUE_DISABLE);
  }

  return result;
}

void FolexDynamixelDriver::resetPosition()
{
  // Set RPM
  double init_rpm = 5.0;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;
  
  for (uint8_t id = 1; id < 13; id++)
  {
    if (dxl_array_.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_MOVING_SPEED, convertRpmToValue(AX_12A, init_rpm));
    }
    else if (dxl_array_.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_PROFILE_VELOCITY, convertRpmToValue(XL430_W250, init_rpm));
    }
    else
    {
      // error
    }   
  }

  // Move to origin
  for (uint8_t id = 1; id < 9; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, 512);
  }

  for (uint8_t id = 9; id < 13; id++)
  {
    writeValue(id, ADDR_XL_GOAL_POSITION, 2048);
  }

  delay(2000);

  std::vector<uint16_t> value;
  value.clear();
  value.push_back(620);
  value.push_back(310);
  value.push_back(413);
  value.push_back(724);
  value.push_back(620);
  value.push_back(310);
  value.push_back(413);
  value.push_back(724);

  for (uint8_t id = 1; id < 9; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 1));
  }
}

bool FolexDynamixelDriver::writeValue(uint8_t id, uint16_t address, uint32_t value)
{
  bool result = false;
  uint8_t dxl_error = 0;

  // AX-12A or XL430
  if (dxl_array_.find(id)->second == AX_12A)
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
  else if (dxl_array_.find(id)->second == XL430_W250)
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
  else
  {
    // error
  }

  return result;
}

uint16_t FolexDynamixelDriver::convertRpmToValue(uint16_t dxl_model, double rpm)
{
  uint16_t value = 0;

  if (dxl_model == AX_12A)
  {
    // AX-12A Units : 0.111 RPM / Value
    value = rpm / 0.111;
  }
  else if (dxl_model == XL430_W250)
  {
    // XL430 Units : 0.229 RPM / Value
    value = rpm / 0.229;
  }
  else
  {
    return false;
  }
  
  return value;
}

/*
*
*   TEST TEST TEST TEST
*
*/
void FolexDynamixelDriver::test()
{
  // Set RPM
  double init_rpm = 5.0;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;
  
  for (uint8_t id = 1; id < 13; id++)
  {
    if (dxl_array_.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_MOVING_SPEED, convertRpmToValue(AX_12A, init_rpm));
    }
    else if (dxl_array_.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_PROFILE_VELOCITY, convertRpmToValue(XL430_W250, init_rpm));
    }
    else
    {
      // error
    }   
  }

  Kinematics kinematics;
  kinematics.solveLegIK(7.425, 0, -129.7739);

  float target_upper_joint = (150 + kinematics.upper_leg_joint) / 0.2932f;
  float target_lower_joint = (150 + kinematics.lower_leg_joint) / 0.2932f;

  std::vector<uint16_t> value;
  value.clear();
  value.push_back(target_upper_joint);
  value.push_back(target_lower_joint);

  for (uint8_t id = 3; id < 5; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 3));
  }

    for (uint8_t id = 7; id < 9; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 7));
  }


  target_upper_joint = (150 - kinematics.upper_leg_joint) / 0.2932f;
  target_lower_joint = (150 - kinematics.lower_leg_joint) / 0.2932f;

  value.clear();
  value.push_back(target_upper_joint);
  value.push_back(target_lower_joint);

  for (uint8_t id = 1; id < 3; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 1));
  }

    for (uint8_t id = 5; id < 7; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 5));
  }
}

void FolexDynamixelDriver::gaitTestFirst()
{
  // Set RPM
  double init_rpm = 15.0;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;
  
  for (uint8_t id = 1; id < 13; id++)
  {
    if (dxl_array_.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_MOVING_SPEED, convertRpmToValue(AX_12A, init_rpm));
    }
    else if (dxl_array_.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_PROFILE_VELOCITY, convertRpmToValue(XL430_W250, init_rpm));
    }
    else
    {
      // error
    }
  }

  Kinematics kinematics;
  
  float temp1_left_target_upper_joint = 0;
  float temp1_left_target_lower_joint = 0;
  float temp1_right_target_upper_joint = 0;
  float temp1_right_target_lower_joint = 0;

  kinematics.solveLegIK(7.425, 0, -129.7739);
  temp1_left_target_upper_joint = (150 - kinematics.upper_leg_joint) / 0.2932f;
  temp1_left_target_lower_joint = (150 - kinematics.lower_leg_joint) / 0.2932f;
  temp1_right_target_upper_joint = (150 + kinematics.upper_leg_joint) / 0.2932f;
  temp1_right_target_lower_joint = (150 + kinematics.lower_leg_joint) / 0.2932f;

  std::vector<uint16_t> value;
  value.clear();
  value.push_back(temp1_left_target_upper_joint);
  value.push_back(temp1_left_target_lower_joint);
  value.push_back(temp1_right_target_upper_joint);
  value.push_back(temp1_right_target_lower_joint);

  // LEFT
  for (uint8_t id = 1; id < 3; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 1));
  }

  for (uint8_t id = 5; id < 7; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 5));
  }

  // RIGHT
  for (uint8_t id = 3; id < 5; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 1));
  }

  for (uint8_t id = 7; id < 9; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 5));
  }
}


void FolexDynamixelDriver::gaitTestSecond()
{
  // Set RPM
  double init_rpm = 15.0;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;
  
  for (uint8_t id = 1; id < 13; id++)
  {
    if (dxl_array_.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_MOVING_SPEED, convertRpmToValue(AX_12A, init_rpm));
    }
    else if (dxl_array_.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_PROFILE_VELOCITY, convertRpmToValue(XL430_W250, init_rpm));
    }
    else
    {
      // error
    }
  }

  Kinematics kinematics;

  float temp2_left_target_upper_joint = 0;
  float temp2_left_target_lower_joint = 0;
  float temp2_right_target_upper_joint = 0;
  float temp2_right_target_lower_joint = 0;

  kinematics.solveLegIK(9.5454, 0, -114.7918);
  temp2_left_target_upper_joint = (150 - kinematics.upper_leg_joint) / 0.2932f;
  temp2_left_target_lower_joint = (150 - kinematics.lower_leg_joint) / 0.2932f;
  temp2_right_target_upper_joint = (150 + kinematics.upper_leg_joint) / 0.2932f;
  temp2_right_target_lower_joint = (150 + kinematics.lower_leg_joint) / 0.2932f;

  std::vector<uint16_t> value;
  value.clear();
  value.push_back(temp2_left_target_upper_joint);
  value.push_back(temp2_left_target_lower_joint);
  value.push_back(temp2_right_target_upper_joint);
  value.push_back(temp2_right_target_lower_joint);

  // LEFT
  for (uint8_t id = 1; id < 3; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 1));
  }

  for (uint8_t id = 5; id < 7; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 5));
  }

  // RIGHT
  for (uint8_t id = 3; id < 5; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 1));
  }

  for (uint8_t id = 7; id < 9; id++)
  {
    writeValue(id, ADDR_AX_GOAL_POSITION, value.at(id - 5));
  }
}

void FolexDynamixelDriver::gaitTrot()
{
  // Set RPM
  double init_rpm = 10.0;
  std::map<uint8_t, uint16_t>::iterator it_dxl_array;
  
  for (uint8_t id = 1; id < 13; id++)
  {
    if (dxl_array_.find(id)->second == AX_12A)
    {
      writeValue(id, ADDR_AX_MOVING_SPEED, convertRpmToValue(AX_12A, init_rpm));
    }
    else if (dxl_array_.find(id)->second == XL430_W250)
    {
      writeValue(id, ADDR_XL_PROFILE_VELOCITY, convertRpmToValue(XL430_W250, init_rpm));
    }
    else
    {
      // error
    }
  }
  
  float temp1_left_target_upper_joint = 0;
  float temp1_left_target_lower_joint = 0;
  float temp1_right_target_upper_joint = 0;
  float temp1_right_target_lower_joint = 0;
  float temp2_left_target_upper_joint = 0;
  float temp2_left_target_lower_joint = 0;
  float temp2_right_target_upper_joint = 0;
  float temp2_right_target_lower_joint = 0;

  Kinematics kinematics;

  kinematics.solveLegIK(7.425, 0, -129.7739);
  temp1_left_target_upper_joint = (150 - kinematics.upper_leg_joint) / 0.2932f;
  temp1_left_target_lower_joint = (150 - kinematics.lower_leg_joint) / 0.2932f;
  temp1_right_target_upper_joint = (150 + kinematics.upper_leg_joint) / 0.2932f;
  temp1_right_target_lower_joint = (150 + kinematics.lower_leg_joint) / 0.2932f;

  kinematics.solveLegIK(9.5454, 0, -114.7918);
  temp2_left_target_upper_joint = (150 - kinematics.upper_leg_joint) / 0.2932f;
  temp2_left_target_lower_joint = (150 - kinematics.lower_leg_joint) / 0.2932f;
  temp2_right_target_upper_joint = (150 + kinematics.upper_leg_joint) / 0.2932f;
  temp2_right_target_lower_joint = (150 + kinematics.lower_leg_joint) / 0.2932f;

  std::vector<uint16_t> value;
  value.clear();
  value.push_back(temp1_left_target_upper_joint);
  value.push_back(temp1_left_target_lower_joint);
  value.push_back(temp1_right_target_upper_joint);
  value.push_back(temp1_right_target_lower_joint);
  value.push_back(temp2_left_target_upper_joint);
  value.push_back(temp2_left_target_lower_joint);
  value.push_back(temp2_right_target_upper_joint);
  value.push_back(temp2_right_target_lower_joint);

  uint8_t dxl_index_one[4] = {1, 2, 7, 8};
  uint8_t dxl_index_two[4] = {5, 6, 3, 4};

  while (true)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      writeValue(dxl_index_one[i], ADDR_AX_GOAL_POSITION, value.at(i));
    }

    for (uint8_t i = 0; i < 4; i++)
    {
      writeValue(dxl_index_two[i], ADDR_AX_GOAL_POSITION, value.at(i + 4));
    }

    delay(1000);

    for (uint8_t i = 0; i < 4; i++)
    {
      writeValue(dxl_index_one[i], ADDR_AX_GOAL_POSITION, value.at(i + 4));
    }

    for (uint8_t i = 0; i < 4; i++)
    {
      writeValue(dxl_index_two[i], ADDR_AX_GOAL_POSITION, value.at(i));
    }

    delay(1000);
  }
}
