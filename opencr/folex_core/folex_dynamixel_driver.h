//

#ifndef FOLEX_DYNAMIXEL_DRIVER_H
#define FOLEX_DYNAMIXEL_DRIVER_H

#include <DynamixelSDK.h>


/* DYNAMIXEL XL430-W250-T */
// http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table
// Model number
#define XL430_W250                1060
// Control table address
#define ADDR_XL_TORQUE_ENABLE     64
#define ADDR_XL_GOAL_VELOCITY     104
#define ADDR_XL_GOAL_POSITION     116
#define ADDR_XL_PRESENT_POSITION  132
// Data byte length
#define LEN_XL_GOAL_POSITION      4

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
// Data byte length
#define LEN_AX_GOAL_POSITION      2
#define LEN_AX_MOVING_SPEED       2

/* DYNAMIXEL ID */
// FRONT-LEFT
#define JOINT_1                   1
#define JOINT_2                   2
#define JOINT_3                   3

// FRONT-RIGHT
#define JOINT_4                   4
#define JOINT_5                   5
#define JOINT_6                   6

// REAR-LEFT
#define JOINT_7                   7
#define JOINT_8                   8
#define JOINT_9                   9

// REAR-RIGHT
#define JOINT_10                  10
#define JOINT_11                  11
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

// Test
#define XL_TEST_POSITION          2048
#define AX_TEST_POSITION          512
#define AX_TEST_RPM               100


class FolexDynamixelDriver
{
private:
  uint32_t baudrate_;
  float protocol_version_;
  std::map<uint8_t, uint16_t> dxl_array_;  // <key, value> <ID, Model Number>
  std::map<uint16_t, uint8_t> dxl_xl_address_array_;  // <key, value> <Address, Size>
  std::map<uint16_t, uint8_t> dxl_ax_address_array_;  // <key, value> <Address, Size>

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteXL_;
  dynamixel::GroupSyncWrite *groupSyncWritePositionAX_;
  dynamixel::GroupSyncWrite *groupSyncWriteRpmAX_;

public:
  FolexDynamixelDriver();
  ~FolexDynamixelDriver();
  bool init();
  void close();
  void addDynamixel(std::string name, std::string parent_name, std::string child_name);
  void moveDynamixel(double j1, double j2, double j3, double time);  // Move Dynamixel manually
  bool writeValue(uint8_t id, uint16_t address, uint32_t value);
  bool setTorque(uint8_t id, bool onoff);
  bool enableDynamixel();
  bool disableDynamixel();
  //bool writePosition();
};

#endif // FOLEX_DYNAMIXEL_DRIVER_H
