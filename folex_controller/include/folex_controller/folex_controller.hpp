//

#ifndef FOLEX_CONTROLLER_HPP
#define FOLEX_CONTROLLER_HPP

#include <ros/ros.h>

#include <std_msgs/String.h>


class FolexController
{
public:
  FolexController(std::string usb_port, std::string baud_rate);
  ~FolexController();

  void publish_callback();

private:
  ros::NodeHandle nh_;
};



# endif  // FOLEX_CONTROLLER_HPP
