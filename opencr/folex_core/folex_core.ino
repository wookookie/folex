//

#include "folex_core.h"

void setup()
{
  // Initialize ROS node handle
  nh.initNode();
  nh.getHardware()->setBaud(1000000);

  folex.initFolex();

}

void loop()
{
  // sendLogMsg();
  // nh.spinOnce();

  folex.gaitTrot();
}


void sendLogMsg()
{
  static bool log_flag = false;
  char log_msg[100];

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      log_flag = true;
    }
    else
    {
      log_flag = false;
    }
  }  
}
