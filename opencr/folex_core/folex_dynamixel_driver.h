//

#ifndef FOLEX_DYNAMIXEL_DRIVER_H
#define FOLEX_DYNAMIXEL_DRIVER_H

#include <DynamixelSDK.h>


// Dynamixel control table address
// AX-12A
#define ADDR_AX_TORQUE_ENABLE     24
#define ADDR_AX_GOAL_POSITION     30
#define ADDR_AX_PRESENT_POSITION  36

#define LEN_AX_GOAL_POSITION      2


// XL430-w250-T
#define ADDR_XL_TORQUE_ENABLE     64
#define ADDR_XL_GOAL_POSITION     116
#define ADDR_XL_PRESENT_POSITION  132


// Dynamixel parameters
#define BAUDRATE                  1000000
#define DEVICENAME                ""          // OpenCR : Empty
#define PROTOCOL_VERSION          1.0


// Dynamixel ID
#define LEFT_FRONT                1

// Test
#define AX_TEST_POSITION          300


class FolexDynamixelDriver
{
public:
  FolexDynamixelDriver();
  ~FolexDynamixelDriver();
  bool init();
  void close();
  bool writePosition();

private:
  uint32_t baudrate_;
  float protocol_version_;

  uint8_t left_front_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWrite_;
};

#endif // FOLEX_DYNAMIXEL_DRIVER_H
