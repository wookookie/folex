//

#ifndef FOLEX_CONTROLLER_H
#define FOLEX_CONTROLLER_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>



class FolexController
{
private:
  ros::NodeHandle nh;

  ros::Subscriber present_joint_value_sub;

  sensor_msgs::JointState joint_state_msg;
  ros::Publisher joint_states_pub;

  std::vector<std::string> joint_names;


public:
  FolexController();
  ~FolexController();

  void publishJointStates(float joint_value[], float joint_speed[]);

  void callbackJointState(const sensor_msgs::JointState::ConstPtr &msg);
};

# endif  // FOLEX_CONTROLLER_H
