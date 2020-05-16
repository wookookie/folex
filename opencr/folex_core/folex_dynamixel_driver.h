//

#ifndef FOLEX_DYNAMIXEL_DRIVER_H
#define FOLEX_DYNAMIXEL_DRIVER_H

#include <DynamixelSDK.h>


/* DYNAMIXEL AX-12A */
// http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table
// Model number
#define AX_12A                    12
// Control table address
#define ADDR_AX_TORQUE_ENABLE     24
#define ADDR_AX_GOAL_POSITION     30
#define ADDR_AX_MOVING_SPEED      32
#define ADDR_AX_PRESENT_POSITION  36
#define ADDR_AX_PRESENT_SPEED     38

/* DYNAMIXEL XL430-W250-T */
// http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table
// Model number
#define XL430_W250                1060
// Control table address
#define ADDR_XL_TORQUE_ENABLE     64
#define ADDR_XL_PROFILE_VELOCITY  112
#define ADDR_XL_GOAL_POSITION     116
#define ADDR_XL_PRESENT_POSITION  132

/* DYNAMIXEL ID */
// FRONT-LEFT
#define JOINT_1                   1
#define JOINT_2                   2
#define JOINT_9                   9

// FRONT-RIGHT
#define JOINT_3                   3
#define JOINT_4                   4
#define JOINT_10                  10

// REAR-LEFT
#define JOINT_5                   5
#define JOINT_6                   6
#define JOINT_11                  11

// REAR-RIGHT
#define JOINT_7                   7
#define JOINT_8                   8
#define JOINT_12                  12

/* POSITION LIMITS */
// To-Do


// Dynamixel parameters
#define BAUDRATE                  1000000
#define DEVICENAME                ""          // OpenCR : Empty
#define PROTOCOL_VERSION          1.0

// Dynamixel torque control
#define TORQUE_DISABLE            0
#define TORQUE_ENABLE             1

// Size
#define BYTE                      1
#define WORD                      2
#define DWORD                     4


class FolexDynamixelDriver
{
private:
  uint32_t baudrate_;
  float protocol_version_;
  std::map<uint8_t, uint16_t> dxl_array_;  // <key, value> <ID, Model Number>
  std::map<uint16_t, uint8_t> dxl_ax_address_array_;  // <key, value> <Address, Size>
  std::map<uint16_t, uint8_t> dxl_xl_address_array_;  // <key, value> <Address, Size>

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

public:
  FolexDynamixelDriver();
  ~FolexDynamixelDriver();
  bool init();
  void initPosition();
  void close();
  void addDynamixel(std::string name, std::string parent_name, std::string child_name);
  bool writeValue(uint8_t id, uint16_t address, uint32_t value);
  bool setTorque(uint8_t id, bool onoff);
  bool enableDynamixel();
  bool disableDynamixel();
  uint16_t convertRpmToValue(uint16_t dxl_model, double rpm);
  //bool writePosition();
};

#endif // FOLEX_DYNAMIXEL_DRIVER_H
