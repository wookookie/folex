//

#include "folex_core.h"

void setup()
{
  // Initialize ROS node handle
  nh.initNode();
  nh.getHardware()->setBaud(1000000);

  diagnosis.init();

  if (dynamixel_driver.init() == false)
  {
    diagnosis.showLedStatus();
  }
  
  if (dynamixel_driver.writePosition() == false)
  {
    diagnosis.showLedStatus();
  }
}

void loop()
{
  startLogMsg();
  delay(500);

  sendLogMsg();
  delay(500);

  nh.spinOnce();
}


// Functions
void sendLogMsg()
{
  char msg[100];

  if (nh.connected())
  {
    sprintf(msg, "Connected to OpenCR Board!");
    nh.loginfo(msg);
  }
}

void startLogMsg()
{
  char msg[100];

  if (nh.connected())
  {
    sprintf(msg, "--- LOOP Start ---");
    nh.loginfo(msg);
  }
}
