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
  //nh.spinOnce();
}
