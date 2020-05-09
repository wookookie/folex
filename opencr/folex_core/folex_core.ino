//

#include "folex_core.h"

void setup()
{
  // Initialize ROS node handle
  nh.initNode();
  nh.getHardware()->setBaud(1000000);

  diagnosis.init();

  dynamixel_driver.init();

}

void loop()
{
  dynamixel_driver.setTorque(1, TORQUE_ENABLE);
  dynamixel_driver.writeValue(1, ADDR_XL_GOAL_POSITION, 2048);

  delay(500);

  dynamixel_driver.setTorque(2, TORQUE_ENABLE);
  dynamixel_driver.writeValue(2, ADDR_AX_GOAL_POSITION, 512);

  delay(500);

  dynamixel_driver.setTorque(3, TORQUE_ENABLE);
  dynamixel_driver.writeValue(3, ADDR_AX_GOAL_POSITION, 512);

  //nh.spinOnce();
}
