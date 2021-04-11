//

#include "folex_controller/folex_controller.hpp"


FolexController::FolexController(std::string usb_port, std::string baud_rate)
{

}

FolexController::~FolexController()
{

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "folex_controller");
  ros::NodeHandle nh;

  // Default settings for USB port and baud rate
  std::string usb_port = "/dev/ttyACM0";
  std::string baud_rate = "1000000";

  // If change USB port or baud rate
  if (argc = 3)
  {
    usb_port = argv[1];
    baud_rate = argv[2];
    printf("USB port : %s\tBaud rate : %s\n", usb_port.c_str(), baud_rate.c_str());
  }
  else
  {
    printf("ERROR : USB port, Baud rate\n");
    return 1;
  }

  FolexController controller(usb_port, baud_rate);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;  
}