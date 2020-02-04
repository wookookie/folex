//

#include <ros/ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "folex_diagnostics");
  ros::NodeHandle nh;

  while (ros::ok())
  {
    ros::spinOnce();
  }
  
  return 0;
}
