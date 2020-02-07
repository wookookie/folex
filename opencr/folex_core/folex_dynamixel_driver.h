//

#ifndef FOLEX_DYNAMIXEL_DRIVER_H
#define FOLEX_DYNAMIXEL_DRIVER_H

#include <DynamixelSDK.h>


/* DYNAMIXEL XL430-W250-T */
// http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table
// Control table address
#define ADDR_XL_TORQUE_ENABLE     64
#define ADDR_XL_GOAL_POSITION     116
#define ADDR_XL_PRESENT_POSITION  132
// Data byte length
#define LEN_XL_GOAL_POSITION      4

/* DYNAMIXEL AX-12A */
// http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table
// Control table address
#define ADDR_AX_TORQUE_ENABLE     24
#define ADDR_AX_GOAL_POSITION     30
#define ADDR_AX_MOVING_SPEED      32
#define ADDR_AX_PRESENT_POSITION  36
#define ADDR_AX_PRESENT_SPEED     38
// Data byte length
#define LEN_AX_GOAL_POSITION      2
#define LEN_AX_MOVING_SPEED       2


// Dynamixel parameters
#define BAUDRATE                  1000000
#define DEVICENAME                ""          // OpenCR : Empty
#define PROTOCOL_VERSION          1.0

// Dynamixel torque control
#define TORQUE_DISABLE            0
#define TORQUE_ENABLE             1

// Dynamixel ID
#define JOINT_1                   1           // XL430-W250-T
#define JOINT_2                   2           // AX-12A
#define JOINT_3                   3           // AX-12A

// Test
#define XL_TEST_POSITION          2048
#define AX_TEST_POSITION          750
#define AX_TEST_RPM               15


class FolexDynamixelDriver
{
public:
  FolexDynamixelDriver();
  ~FolexDynamixelDriver();
  bool init();
  void close();
  bool setTorque(bool onoff);
  bool writePosition();

private:
  uint32_t baudrate_;
  float protocol_version_;
  bool torque_;

  uint8_t joint_1_id_;
  uint8_t joint_2_id_;
  uint8_t joint_3_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteXL_;
  dynamixel::GroupSyncWrite *groupSyncWritePositionAX_;
  dynamixel::GroupSyncWrite *groupSyncWriteRpmAX_;
};

#endif // FOLEX_DYNAMIXEL_DRIVER_H
