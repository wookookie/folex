//

#include "folex_core.h"

void setup()
{
  // Initialize ROS node handle
  nh.initNode();
  nh.getHardware()->setBaud(1000000);

  nh.advertise(chatter_pub);

  
  //folex.initFolex();
}

void loop()
{
  std::string message;
  message = "TEST TEST";

  if (millis() - pre_time >= 50)
  {
    msg.data = message.c_str();
    chatter_pub.publish(&msg);
    pre_time = millis();
  }
  
  nh.spinOnce();
}
